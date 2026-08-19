#pragma once

#include "motion/trajectory/joint_trajectory.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot::motion
{

    // ============================================================
    // SampledJointTrajectory
    // ============================================================
    //
    // 表示一条已经由外部系统给出离散时间采样点的轨迹。
    //
    // 例如:
    //
    //     t = 0.0 s → q0
    //     t = 0.1 s → q1
    //     t = 0.2 s → q2
    //     t = 0.3 s → q3
    //
    // SampledJointTrajectory 将这些离散点
    // 适配成统一的 JointTrajectory<N> 接口。
    //
    // ------------------------------------------------------------
    //
    // 典型数据来源:
    //
    //     - 外部 motion planner
    //     - 网络接收到的 timed path
    //     - 文件中保存的轨迹
    //     - 其他机器人规划系统
    //
    // ------------------------------------------------------------
    //
    // 当前第一版采用:
    //
    //     piecewise linear interpolation
    //
    // 对两个相邻位置 sample:
    //
    //     qa @ ta
    //     qb @ tb
    //
    // 在这两个采样点之间:
    //
    //     position:
    //         线性插值
    //
    //     velocity:
    //         (qb - qa) / (tb - ta)
    //
    //     acceleration:
    //         0
    //
    // ------------------------------------------------------------
    //
    // 注意:
    //
    // 这种轨迹在 waypoint 处速度可能不连续。
    //
    // 因此它不是我们机械臂平滑 MoveJ 的首选实现。
    // 我们当前正常机械臂运动应该优先使用:
    //
    //     QuinticJointTrajectory
    //
    // SampledJointTrajectory 的主要价值是：
    //
    //     把已经离散化的外部轨迹
    //     适配成 JointTrajectory 接口。
    //
    // ============================================================

    template <std::size_t N> class SampledJointTrajectory final : public JointTrajectory<N>
    {
    public:
        using Base = JointTrajectory<N>;

        using Duration = typename Base::Duration;

        using Point = typename Base::Point;

        using JointValues = std::array<double, N>;

        struct Sample
        {
            // 相对整条轨迹起点的时间。
            Duration time_from_start{};

            // 当前采样时刻的关节位置。
            JointValues position{};
        };

    public:
        explicit SampledJointTrajectory(const std::vector<Sample> &samples) : samples_(samples)
        {
            if (samples_.size() < 2)
            {
                throw std::invalid_argument("Sampled trajectory requires at least two samples");
            }

            if (samples_.front().time_from_start != Duration::zero())
            {
                throw std::invalid_argument("First sample time must be zero");
            }

            // 时间必须严格递增。
            for (std::size_t i = 0; i + 1 < samples_.size(); ++i)
            {
                if (samples_[i + 1].time_from_start <= samples_[i].time_from_start)
                {
                    throw std::invalid_argument("Sample times must be strictly increasing");
                }
            }

            duration_ = samples_.back().time_from_start;
        }

    public:
        Point sample(Duration elapsed) const override
        {
            // ----------------------------------------------------
            // Before start
            // ----------------------------------------------------

            if (elapsed <= Duration::zero())
            {
                Point point;

                point.position = samples_.front().position;

                return point;
            }

            // ----------------------------------------------------
            // After trajectory end
            // ----------------------------------------------------

            if (elapsed >= duration_)
            {
                Point point;

                point.position = samples_.back().position;

                point.finished = true;

                return point;
            }

            // ----------------------------------------------------
            // Find active sample interval
            // ----------------------------------------------------

            for (std::size_t i = 0; i + 1 < samples_.size(); ++i)
            {
                const auto &start = samples_[i];

                const auto &goal = samples_[i + 1];

                if (elapsed < goal.time_from_start)
                {
                    const Duration segment_duration = goal.time_from_start - start.time_from_start;

                    const Duration local_elapsed = elapsed - start.time_from_start;

                    const double segment_seconds = std::chrono::duration<double>(segment_duration).count();

                    const double local_seconds = std::chrono::duration<double>(local_elapsed).count();

                    // alpha:
                    //
                    //     0.0 → segment start
                    //     1.0 → segment end
                    const double alpha = local_seconds / segment_seconds;

                    Point point;

                    for (std::size_t joint = 0; joint < N; ++joint)
                    {
                        const double delta = goal.position[joint] - start.position[joint];

                        // ------------------------------
                        // Position
                        // ------------------------------

                        point.position[joint] = start.position[joint] + alpha * delta;

                        // ------------------------------
                        // Velocity
                        // ------------------------------
                        //
                        // 线性插值段内速度为常数。
                        point.velocity[joint] = delta / segment_seconds;

                        // ------------------------------
                        // Acceleration
                        // ------------------------------
                        //
                        // 在线性 segment 内：
                        //
                        //     dq = constant
                        //
                        // 所以：
                        //
                        //     ddq = 0
                        point.acceleration[joint] = 0.0;
                    }

                    return point;
                }
            }

            // 理论上不会进入这里，
            // 仅作为安全 fallback。
            Point point;

            point.position = samples_.back().position;

            point.finished = true;

            return point;
        }

        Duration duration() const noexcept override
        {
            return duration_;
        }

    private:
        std::vector<Sample> samples_;

        Duration duration_{};
    };

} // namespace robot::motion