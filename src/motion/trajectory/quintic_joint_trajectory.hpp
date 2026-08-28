#pragma once

#include "motion/trajectory/joint_trajectory.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>

namespace robot::motion
{

// ============================================================
// QuinticJointTrajectory
// ============================================================
//
// 单段五次多项式关节轨迹。
//
// 它描述：
//
//     起始关节状态
//           ↓
//      五次多项式
//           ↓
//     目标关节状态
//
// 对每个关节分别生成：
//
//     q(t)     position
//     dq(t)    velocity
//     ddq(t)   acceleration
//
// 所有关节共享同一个 duration，因此整组关节同时开始、同时结束。
//
// ------------------------------------------------------------
//
// QuinticJointTrajectory 只负责关节轨迹数学计算。
//
// 它不负责：
//
//     - FK / IK
//     - 笛卡尔空间规划
//     - 碰撞检测
//     - Executor
//     - Controller
//     - CAN
//     - YAML
//     - 自动决定运动时间
//
// ------------------------------------------------------------
//
// 最简单的 MoveJ 数据链：
//
//     q_start
//        ↓
// QuinticJointTrajectory
//        ↓
// JointTrajectoryPoint<N>
//        ↓
// RobotControlExecutor
//
// ============================================================

template <std::size_t N>
class QuinticJointTrajectory final : public JointTrajectory<N>
{
public:
    using Base = JointTrajectory<N>;
    using Duration = typename Base::Duration;
    using Point = typename Base::Point;
    using JointValues = std::array<double, N>;

    // ========================================================
    // BoundaryState
    // ========================================================
    //
    // 五次多项式有 6 个系数，因此可以同时约束：
    //
    // 起点：
    //     position
    //     velocity
    //     acceleration
    //
    // 终点：
    //     position
    //     velocity
    //     acceleration
    //
    // 最普通的 point-to-point 运动通常使用：
    //
    //     start velocity     = 0
    //     start acceleration = 0
    //
    //     goal velocity      = 0
    //     goal acceleration  = 0
    //
    // 这样可以实现平稳启动和平稳停止。
    //
    // 保留非零 velocity / acceleration，是为了以后多段轨迹平滑连接。
    struct BoundaryState
    {
        JointValues position{};
        JointValues velocity{};
        JointValues acceleration{};
    };

public:
    // ========================================================
    // Full constructor
    // ========================================================
    //
    // 根据完整的起点状态、终点状态和 duration，在构造阶段一次性计算所有关节
    // 的多项式系数。
    //
    // 系数在构造时计算，而不是每次 sample() 时重新求解。因为 sample() 会在
    // Executor 控制线程中频繁调用，应尽可能简单、快速、无阻塞。
    QuinticJointTrajectory(const BoundaryState &start,
                           const BoundaryState &goal,
                           Duration duration)
        : duration_(duration),
          duration_seconds_(std::chrono::duration<double>(duration).count())
    {
        if (duration_ <= Duration::zero())
        {
            throw std::invalid_argument(
                "Quintic trajectory duration must be positive");
        }

        const double T = duration_seconds_;
        const double T2 = T * T;
        const double T3 = T2 * T;
        const double T4 = T3 * T;
        const double T5 = T4 * T;

        // 每个关节独立计算一组六个系数。
        for (std::size_t i = 0; i < N; ++i)
        {
            const double q0 = start.position[i];
            const double qf = goal.position[i];
            const double v0 = start.velocity[i];
            const double vf = goal.velocity[i];
            const double acc0 = start.acceleration[i];
            const double accf = goal.acceleration[i];
            auto &a = coefficients_[i];

            // q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
            a[0] = q0;
            a[1] = v0;
            a[2] = 0.5 * acc0;
            a[3] =
                (20.0 * (qf - q0) - (8.0 * vf + 12.0 * v0) * T -
                 (3.0 * acc0 - accf) * T2) /
                (2.0 * T3);
            a[4] =
                (30.0 * (q0 - qf) + (14.0 * vf + 16.0 * v0) * T +
                 (3.0 * acc0 - 2.0 * accf) * T2) /
                (2.0 * T4);
            a[5] =
                (12.0 * (qf - q0) - (6.0 * vf + 6.0 * v0) * T -
                 (acc0 - accf) * T2) /
                (2.0 * T5);
        }
    }

    // ========================================================
    // Simple point-to-point constructor
    // ========================================================
    //
    // 最常见的 MoveJ 使用方式。只给 q_start、q_goal 和 duration，默认起点、终点
    // 的速度和加速度都为 0。保留这个构造函数是为了方便第一版 Executor 测试。
    QuinticJointTrajectory(const JointValues &start_position,
                           const JointValues &goal_position,
                           Duration duration)
        : QuinticJointTrajectory(
              BoundaryState{start_position, {}, {}},
              BoundaryState{goal_position, {}, {}},
              duration)
    {
    }

public:
    // ========================================================
    // sample
    // ========================================================
    //
    // elapsed 表示轨迹从开始执行到现在经过了多少时间。
    //
    // elapsed <= 0：返回起点状态。
    // 0 < elapsed < duration：正常计算当前轨迹状态。
    // elapsed >= duration：时间 clamp 到终点，finished = true。
    //
    // Executor 可以一直安全调用 sample()，不需要担心 elapsed 稍微超过 duration。
    Point sample(Duration elapsed) const override
    {
        const bool finished = elapsed >= duration_;
        double t = 0.0;

        if (elapsed > Duration::zero())
        {
            if (finished)
            {
                // 超过结束时间时固定在终点。
                t = duration_seconds_;
            }
            else
            {
                t = std::chrono::duration<double>(elapsed).count();
            }
        }

        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;

        Point point;
        point.finished = finished;

        for (std::size_t i = 0; i < N; ++i)
        {
            const auto &a = coefficients_[i];

            // Position: q(t)
            point.position[i] =
                a[0] + a[1] * t + a[2] * t2 + a[3] * t3 + a[4] * t4 +
                a[5] * t5;

            // Velocity: dq(t) / dt
            point.velocity[i] =
                a[1] + 2.0 * a[2] * t + 3.0 * a[3] * t2 +
                4.0 * a[4] * t3 + 5.0 * a[5] * t4;

            // Acceleration: d²q(t) / dt²
            point.acceleration[i] =
                2.0 * a[2] + 6.0 * a[3] * t + 12.0 * a[4] * t2 +
                20.0 * a[5] * t3;
        }

        return point;
    }

    // 返回整条轨迹持续时间。
    Duration duration() const noexcept override
    {
        return duration_;
    }

private:
    // 一个关节对应 a0 ~ a5 六个五次多项式系数。
    using Coefficients = std::array<double, 6>;

    // N 个关节分别保存自己的多项式系数。
    std::array<Coefficients, N> coefficients_{};

    // 对外保持 steady_clock::duration，与 JointTrajectory 接口一致。
    Duration duration_{};

    // 内部多项式计算统一使用秒。
    double duration_seconds_{0.0};
};

} // namespace robot::motion
