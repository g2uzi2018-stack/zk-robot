#pragma once

#include "motion/kinematics/joint_position_constraints.hpp"
#include "motion/planning/joint_acceleration_limits.hpp"
#include "motion/planning/velocity_time_parameterization.hpp"
#include "motion/trajectory/joint_trajectory.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>
#include <iomanip>
#include <iostream>

namespace robot::motion::planning
{

    /**
     * CubicJointTrajectory 的动态约束倍率。
     *
     * velocity_scaling：
     *     实际允许速度 =
     *         JointVelocityLimits 中的最大速度
     *         * velocity_scaling
     *
     * acceleration_scaling：
     *     实际允许加速度 =
     *         JointAccelerationLimits 中的最大加速度
     *         * acceleration_scaling
     *
     * 二者都必须位于 (0, 1]。
     *
     * 本类不会主动把前一级的时间表缩短。
     * 如果连续三次样条会导致速度或加速度超限，
     * 只会把整条时间轴统一拉长。
     */
    struct CubicJointTrajectoryOptions
    {
        double velocity_scaling{1.0};
        double acceleration_scaling{1.0};
    };

    /**
     * 连续、可按任意时间采样的关节三次样条轨迹。
     *
     * 上游输入：
     *
     *     VelocityTimedJointPath
     *
     *     例如：
     *
     *         t0 -> q0
     *         t1 -> q1
     *         t2 -> q2
     *         ...
     *
     *     再配合：
     *
     *         JointPositionConstraints
     *         JointVelocityLimits
     *         JointAccelerationLimits
     *
     * 下游：
     *
     *     Executor / Controller 可以通过：
     *
     *         sample(elapsed)
     *
     *     查询任意执行时刻对应的：
     *
     *         position
     *         velocity
     *         acceleration
     *
     * ============================================================
     * 主要作用
     * ============================================================
     *
     * 前一级 VelocityTimedJointPath 只规定：
     *
     *     “什么时候经过哪些离散 q waypoint”
     *
     * 本类进一步把这些离散 waypoint 连接成真正的连续 q(t)。
     *
     * 它保证：
     *
     *     1. 精确经过输入的所有 q waypoint；
     *     2. 起点速度为 0；
     *     3. 终点速度为 0；
     *     4. 内部 waypoint 的位置连续；
     *     5. 内部 waypoint 的速度连续；
     *     6. 内部 waypoint 的加速度连续；
     *     7. 连续轨迹不越关节位置范围；
     *     8. 连续轨迹的速度不超过配置限制；
     *     9. 连续轨迹的加速度不超过配置限制。
     *
     * 如果原来的时间太短，会整体拉长时间。
     *
     * ============================================================
     * 当前仍然不负责
     * ============================================================
     *
     *     - jerk 限制；
     *     - 碰撞检测；
     *     - TCP 笛卡尔速度/加速度限制；
     *     - 检查两个 IK waypoint 中间的 TCP
     *       是否严格贴着 CartesianLinePath；
     *     - CAN / Controller / Executor。
     */
    template <std::size_t N>
    class CubicJointTrajectory final
        : public JointTrajectory<N>
    {
    public:
        using Base = JointTrajectory<N>;
        using Duration = typename Base::Duration;
        using Point = typename Base::Point;
        using JointVector = std::array<double, N>;

    private:
        /**
         * 一个连续轨迹 segment。
         *
         * 局部时间：
         *
         *     tau = 当前时间 - segment.start_time
         *
         * tau 范围：
         *
         *     [0, duration]
         *
         * 每一个关节使用三次多项式：
         *
         *     q(tau)
         *       = a
         *       + b*tau
         *       + c*tau^2
         *       + d*tau^3
         *
         * 所以同一个 Segment 同时保存 N 个关节的系数。
         */
        struct Segment
        {
            double start_time{0.0};
            double duration{0.0};

            JointVector a{};
            JointVector b{};
            JointVector c{};
            JointVector d{};
        };

    public:
        /**
         * 构造完整连续轨迹。
         *
         * model：
         *     提供关节名称、q 顺序等模型信息。
         *
         * position_constraints：
         *     检查所有 waypoint 和连续样条中间值
         *     都不越过关节位置范围。
         *
         * velocity_limits：
         *     每个 q 对应的最大关节速度。
         *
         * acceleration_limits：
         *     每个 q 对应的最大关节加速度。
         *
         * source：
         *     前一步已经得到的带时间关节 waypoint。
         *
         * options：
         *     速度 / 加速度倍率。
         */
        CubicJointTrajectory(
            const ValidatedUrdfChain<N> &model,
            const JointPositionConstraints<N> &position_constraints,
            const JointVelocityLimits<N> &velocity_limits,
            const JointAccelerationLimits<N> &acceleration_limits,
            const VelocityTimedJointPath<N> &source,
            const CubicJointTrajectoryOptions &options = {})
            : options_(options)
        {
            /*
             * ----------------------------------------------------
             * 1. 检查倍率参数
             * ----------------------------------------------------
             */
            if (!std::isfinite(options_.velocity_scaling) ||
                options_.velocity_scaling <= 0.0 ||
                options_.velocity_scaling > 1.0 ||
                !std::isfinite(options_.acceleration_scaling) ||
                options_.acceleration_scaling <= 0.0 ||
                options_.acceleration_scaling > 1.0)
            {
                throw std::invalid_argument(
                    "Invalid cubic trajectory scaling");
            }

            /*
             * ----------------------------------------------------
             * 2. 确认所有限制对象与当前 model 使用同一个 q 顺序
             * ----------------------------------------------------
             */
            position_constraints.requireJointOrder(model);
            velocity_limits.requireJointOrder(model);
            acceleration_limits.requireJointOrder(model);

            /*
             * ----------------------------------------------------
             * 3. 输入路径必须已经完整完成
             * ----------------------------------------------------
             */
            if (!source.completed())
            {
                throw std::invalid_argument(
                    "Cubic trajectory requires completed timed path");
            }

            if (source.waypoints.empty())
            {
                throw std::invalid_argument(
                    "Cubic trajectory requires at least one waypoint");
            }

            /*
             * 保存动态限制。
             */
            velocity_limits_ =
                velocity_limits.maxVelocity();

            acceleration_limits_ =
                acceleration_limits.maxAcceleration();

            joint_names_ =
                velocity_limits.jointNames();

            /*
             * ----------------------------------------------------
             * 4. 保存输入 waypoint
             * ----------------------------------------------------
             */
            positions_.reserve(
                source.waypoints.size());

            times_.reserve(
                source.waypoints.size());

            for (std::size_t i = 0;
                 i < source.waypoints.size();
                 ++i)
            {
                const auto &waypoint =
                    source.waypoints[i];

                const double time =
                    waypoint.time_from_start.count();

                if (!std::isfinite(time))
                {
                    throw std::invalid_argument(
                        "Non-finite waypoint time");
                }

                /*
                 * 第一个 waypoint 必须从 t=0 开始。
                 */
                if (i == 0)
                {
                    if (std::abs(time) > 1e-15)
                    {
                        throw std::invalid_argument(
                            "First timed waypoint must start at zero");
                    }
                }
                else if (time <= times_.back())
                {
                    /*
                     * 后续时间必须严格递增。
                     */
                    throw std::invalid_argument(
                        "Timed waypoints must be strictly increasing");
                }

                /*
                 * 所有 q 必须是有限数值。
                 */
                for (const double q :
                     waypoint.q)
                {
                    if (!std::isfinite(q))
                    {
                        throw std::invalid_argument(
                            "Non-finite joint waypoint");
                    }
                }

                /*
                 * 离散 waypoint 本身不能越位置限制。
                 */
                if (!position_constraints.contains(
                        waypoint.q))
                {
                    throw std::invalid_argument(
                        "Timed waypoint violates position bounds");
                }

                times_.push_back(time);

                positions_.push_back(
                    waypoint.q);
            }

            /*
             * ----------------------------------------------------
             * 5. 零运动路径
             * ----------------------------------------------------
             *
             * 只有一个 waypoint：
             *
             *     duration = 0
             *
             * 不需要建立任何 segment。
             */
            if (positions_.size() == 1)
            {
                total_seconds_ = 0.0;
                duration_ = Duration::zero();

                return;
            }

            /*
             * ----------------------------------------------------
             * 6. 使用前一级时间表构造第一版 spline
             * ----------------------------------------------------
             */
            buildSpline();

            /*
             * ----------------------------------------------------
             * 7. 测量连续 spline 真正出现的速度 / 加速度峰值
             * ----------------------------------------------------
             *
             * 注意：
             *
             * 上一级只检查：
             *
             *     delta_q / delta_t
             *
             * 但三次样条内部瞬时速度可能比这个值更大，
             * 所以必须检查真正连续曲线。
             */
            const PeakRatios initial =
                measurePeakRatios();

            /*
             * ----------------------------------------------------
             * 8. 计算整条轨迹需要拉长多少倍
             * ----------------------------------------------------
             *
             * 时间整体乘以 scale 后：
             *
             *     velocity     会除以 scale
             *     acceleration 会除以 scale^2
             *
             * 因此：
             *
             *     速度超限 ratio_v 倍
             *         → 时间至少乘 ratio_v
             *
             *     加速度超限 ratio_a 倍
             *         → 时间至少乘 sqrt(ratio_a)
             */
            time_scale_ =
                std::max(
                    1.0,
                    std::max(
                        initial.velocity_ratio,
                        std::sqrt(
                            initial.acceleration_ratio)));

            if (!std::isfinite(time_scale_))
            {
                throw std::runtime_error(
                    "Cubic trajectory time scale overflow");
            }

            /*
             * 只允许把时间拉长，
             * 不主动把前一级时间表缩短。
             */
            if (time_scale_ >
                1.0 + 1e-12)
            {
                for (double &time :
                     times_)
                {
                    time *= time_scale_;
                }

                /*
                 * 时间改变以后重新构造样条系数。
                 */
                buildSpline();
            }

            /*
             * ----------------------------------------------------
             * 9. 检查连续样条中间有没有越过关节位置范围
             * ----------------------------------------------------
             *
             * 即使 q0 和 q1 都合法，
             * cubic spline 也可能在中间 overshoot。
             *
             * 单纯放慢时间无法消除几何 overshoot，
             * 所以必须独立检查。
             */
            validateContinuousPositionBounds(
                position_constraints);

            /*
             * ----------------------------------------------------
             * 10. 最终再次测量动态限制
             * ----------------------------------------------------
             */
            const PeakRatios final =
                measurePeakRatios();

            max_velocity_ratio_ =
                final.velocity_ratio;

            max_acceleration_ratio_ =
                final.acceleration_ratio;

            limiting_velocity_joint_ =
                final.velocity_joint;

            limiting_acceleration_joint_ =
                final.acceleration_joint;

            if (max_velocity_ratio_ >
                    1.0 + 1e-10 ||
                max_acceleration_ratio_ >
                    1.0 + 1e-10)
            {
                throw std::runtime_error(
                    "Cubic trajectory still violates dynamic limits");
            }

            /*
             * ----------------------------------------------------
             * 11. 保存最终轨迹时长
             * ----------------------------------------------------
             */
            total_seconds_ =
                times_.back();

            duration_ =
                std::chrono::duration_cast<Duration>(
                    std::chrono::duration<double>{
                        total_seconds_});
        }

        /**
         * JointTrajectory 的核心接口。
         *
         * 输入：
         *
         *     elapsed
         *
         * 表示：
         *
         *     从这条轨迹开始执行到现在经过了多久。
         *
         * 输出：
         *
         *     Point
         *
         * 内含：
         *
         *     position[N]
         *     velocity[N]
         *     acceleration[N]
         *     finished
         *
         * 所以它真正回答：
         *
         *     “现在这个时刻，7 个关节应该是什么状态？”
         */
        Point sample(
            const Duration elapsed) const override
        {
            Point point;

            /*
             * ----------------------------------------------------
             * 零长度轨迹
             * ----------------------------------------------------
             */
            if (segments_.empty())
            {
                point.position =
                    positions_.front();

                /*
                 * velocity / acceleration
                 * 默认初始化为 0。
                 */
                point.finished =
                    true;

                return point;
            }

            const double seconds =
                std::chrono::duration<double>(
                    elapsed)
                    .count();

            if (!std::isfinite(seconds))
            {
                throw std::invalid_argument(
                    "Trajectory sample time must be finite");
            }

            /*
             * ----------------------------------------------------
             * 轨迹开始之前 / 正好开始
             * ----------------------------------------------------
             */
            if (seconds <= 1e-9)
            {
                return evaluateSegment(
                    segments_.front(),
                    0.0,
                    false);
            }

            /*
             * ----------------------------------------------------
             * 到达或超过终点
             * ----------------------------------------------------
             */
            std::cerr
                << std::setprecision(17)
                << "sample seconds="
                << seconds
                << " total_seconds="
                << total_seconds_
                << std::endl;
            if (seconds >= total_seconds_ - 1e-9)
            {
                const auto &last =
                    segments_.back();

                return evaluateSegment(
                    last,
                    last.duration,
                    true);
            }

            /*
             * ----------------------------------------------------
             * 找到当前时间属于哪个 segment
             * ----------------------------------------------------
             *
             * 例如：
             *
             *     segment0: 0.000 ~ 0.020
             *     segment1: 0.020 ~ 0.041
             *
             * 查询：
             *
             *     t = 0.030
             *
             * 就会定位到 segment1。
             */
            const auto found =
                std::lower_bound(
                    segments_.begin(),
                    segments_.end(),
                    seconds,
                    [](const Segment &segment,
                       const double time)
                    {
                        return segment.start_time +
                                   segment.duration <
                               time;
                    });

            if (found ==
                segments_.end())
            {
                throw std::logic_error(
                    "Could not resolve cubic trajectory segment");
            }

            /*
             * 转换成该 segment 内部局部时间 tau。
             */
            const double tau =
                std::clamp(
                    seconds -
                        found->start_time,
                    0.0,
                    found->duration);

            return evaluateSegment(
                *found,
                tau,
                false);
        }

        /**
         * JointTrajectory 接口。
         *
         * 返回整条最终平滑轨迹需要的总时间。
         */
        Duration duration() const noexcept override
        {
            return duration_;
        }

        /**
         * 返回这一层相对于上一层时间表
         * 整体放慢了多少倍。
         *
         * 例如：
         *
         *     1.0
         *         没有额外减速。
         *
         *     2.0
         *         总体时间被拉长成原来的 2 倍。
         */
        double timeScale() const noexcept
        {
            return time_scale_;
        }

        /**
         * 最终连续轨迹中：
         *
         *     最大实际速度 / 允许速度
         */
        double maxVelocityRatio() const noexcept
        {
            return max_velocity_ratio_;
        }

        /**
         * 最终连续轨迹中：
         *
         *     最大实际加速度 / 允许加速度
         */
        double maxAccelerationRatio() const noexcept
        {
            return max_acceleration_ratio_;
        }

        /**
         * 返回最终最接近速度上限的关节名称。
         */
        const std::string &
        limitingVelocityJoint() const noexcept
        {
            return limiting_velocity_joint_;
        }

        /**
         * 返回最终最接近加速度上限的关节名称。
         */
        const std::string &
        limitingAccelerationJoint() const noexcept
        {
            return limiting_acceleration_joint_;
        }

    private:
        /**
         * 内部动态峰值检查结果。
         */
        struct PeakRatios
        {
            double velocity_ratio{0.0};
            double acceleration_ratio{0.0};

            std::string velocity_joint;
            std::string acceleration_joint;
        };

        /**
         * 根据当前：
         *
         *     times_
         *     positions_
         *
         * 构造 clamped cubic spline。
         *
         * 边界条件：
         *
         *     q_dot(start) = 0
         *     q_dot(goal)  = 0
         *
         * 中间 waypoint：
         *
         *     位置连续
         *     速度连续
         *     加速度连续
         *
         * 注意：
         *
         * 中间 waypoint 的速度不会被设成 0。
         * 所以机器人不会经过每个 IK waypoint 时都停下来。
         */
        void buildSpline()
        {
            segments_.clear();

            const std::size_t waypoint_count =
                positions_.size();

            if (waypoint_count <= 1)
            {
                return;
            }

            const std::size_t segment_count =
                waypoint_count - 1;

            /*
             * h[i]：
             *
             *     waypoint i
             *          ↓
             *       duration
             *          ↓
             *     waypoint i+1
             */
            std::vector<double> h(
                segment_count);

            for (std::size_t i = 0;
                 i < segment_count;
                 ++i)
            {
                h[i] =
                    times_[i + 1] -
                    times_[i];

                if (!std::isfinite(h[i]) ||
                    h[i] <= 0.0)
                {
                    throw std::invalid_argument(
                        "Invalid cubic spline segment duration");
                }
            }

            /*
             * 建立每个 segment 的公共时间信息。
             */
            segments_.resize(
                segment_count);

            for (std::size_t segment = 0;
                 segment < segment_count;
                 ++segment)
            {
                segments_[segment].start_time =
                    times_[segment];

                segments_[segment].duration =
                    h[segment];
            }

            /*
             * 每个关节独立计算 spline 系数，
             * 但是全部关节共享同样的 waypoint 时间。
             */
            for (std::size_t joint = 0;
                 joint < N;
                 ++joint)
            {
                /*
                 * 三对角方程组。
                 *
                 * 求的是每个 waypoint 上的二阶导数。
                 */
                std::vector<double> lower(
                    waypoint_count,
                    0.0);

                std::vector<double> diagonal(
                    waypoint_count,
                    0.0);

                std::vector<double> upper(
                    waypoint_count,
                    0.0);

                std::vector<double> rhs(
                    waypoint_count,
                    0.0);

                /*
                 * ------------------------------------------------
                 * 起点：
                 *
                 * q_dot(start) = 0
                 * ------------------------------------------------
                 */
                diagonal[0] =
                    2.0 * h[0];

                upper[0] =
                    h[0];

                rhs[0] =
                    6.0 *
                    ((positions_[1][joint] -
                      positions_[0][joint]) /
                     h[0]);

                /*
                 * ------------------------------------------------
                 * 中间 waypoint
                 * ------------------------------------------------
                 */
                for (std::size_t i = 1;
                     i + 1 < waypoint_count;
                     ++i)
                {
                    const double left_h =
                        h[i - 1];

                    const double right_h =
                        h[i];

                    lower[i] =
                        left_h;

                    diagonal[i] =
                        2.0 *
                        (left_h +
                         right_h);

                    upper[i] =
                        right_h;

                    const double left_slope =
                        (positions_[i][joint] -
                         positions_[i - 1][joint]) /
                        left_h;

                    const double right_slope =
                        (positions_[i + 1][joint] -
                         positions_[i][joint]) /
                        right_h;

                    rhs[i] =
                        6.0 *
                        (right_slope -
                         left_slope);
                }

                /*
                 * ------------------------------------------------
                 * 终点：
                 *
                 * q_dot(goal) = 0
                 * ------------------------------------------------
                 */
                const std::size_t last =
                    waypoint_count - 1;

                lower[last] =
                    h[last - 1];

                diagonal[last] =
                    2.0 *
                    h[last - 1];

                rhs[last] =
                    -6.0 *
                    ((positions_[last][joint] -
                      positions_[last - 1][joint]) /
                     h[last - 1]);

                /*
                 * ------------------------------------------------
                 * Thomas algorithm
                 * ------------------------------------------------
                 *
                 * 解三对角线性系统。
                 */
                for (std::size_t i = 1;
                     i < waypoint_count;
                     ++i)
                {
                    if (!std::isfinite(
                            diagonal[i - 1]) ||
                        std::abs(
                            diagonal[i - 1]) <=
                            1e-18)
                    {
                        throw std::runtime_error(
                            "Singular cubic spline system");
                    }

                    const double factor =
                        lower[i] /
                        diagonal[i - 1];

                    diagonal[i] -=
                        factor *
                        upper[i - 1];

                    rhs[i] -=
                        factor *
                        rhs[i - 1];
                }

                std::vector<double> second(
                    waypoint_count,
                    0.0);

                if (!std::isfinite(
                        diagonal[last]) ||
                    std::abs(
                        diagonal[last]) <=
                        1e-18)
                {
                    throw std::runtime_error(
                        "Singular cubic spline system");
                }

                second[last] =
                    rhs[last] /
                    diagonal[last];

                /*
                 * 回代。
                 */
                for (std::size_t reverse = last;
                     reverse-- > 0;)
                {
                    if (!std::isfinite(
                            diagonal[reverse]) ||
                        std::abs(
                            diagonal[reverse]) <=
                            1e-18)
                    {
                        throw std::runtime_error(
                            "Singular cubic spline system");
                    }

                    second[reverse] =
                        (rhs[reverse] -
                         upper[reverse] *
                             second[reverse + 1]) /
                        diagonal[reverse];
                }

                /*
                 * ------------------------------------------------
                 * 转换成每个 segment 的：
                 *
                 * a b c d
                 * ------------------------------------------------
                 */
                for (std::size_t segment = 0;
                     segment < segment_count;
                     ++segment)
                {
                    const double dt =
                        h[segment];

                    const double q0 =
                        positions_[segment][joint];

                    const double q1 =
                        positions_[segment + 1][joint];

                    const double m0 =
                        second[segment];

                    const double m1 =
                        second[segment + 1];

                    Segment &output =
                        segments_[segment];

                    output.a[joint] =
                        q0;

                    output.b[joint] =
                        (q1 - q0) /
                            dt -
                        dt *
                            (2.0 * m0 +
                             m1) /
                            6.0;

                    output.c[joint] =
                        0.5 * m0;

                    output.d[joint] =
                        (m1 - m0) /
                        (6.0 * dt);
                }
            }
        }

        /**
         * 检查整条连续 spline 中真正出现的：
         *
         *     最大速度比例
         *     最大加速度比例
         *
         * 不是只检查 waypoint。
         */
        PeakRatios measurePeakRatios() const
        {
            PeakRatios result;

            for (const auto &segment :
                 segments_)
            {
                const double h =
                    segment.duration;

                for (std::size_t joint = 0;
                     joint < N;
                     ++joint)
                {
                    const double allowed_velocity =
                        velocity_limits_[joint] *
                        options_.velocity_scaling;

                    const double allowed_acceleration =
                        acceleration_limits_[joint] *
                        options_.acceleration_scaling;

                    if (!std::isfinite(
                            allowed_velocity) ||
                        allowed_velocity <= 0.0 ||
                        !std::isfinite(
                            allowed_acceleration) ||
                        allowed_acceleration <= 0.0)
                    {
                        throw std::runtime_error(
                            "Invalid dynamic limit");
                    }

                    /*
                     * 速度：
                     *
                     * q_dot(tau)
                     *     =
                     * b
                     * + 2c*tau
                     * + 3d*tau²
                     *
                     * 最大绝对速度可能出现在：
                     *
                     *     segment 起点
                     *     segment 终点
                     *     acceleration == 0 的内部点
                     */
                    double peak_velocity =
                        std::max(
                            std::abs(
                                evaluateVelocity(
                                    segment,
                                    joint,
                                    0.0)),
                            std::abs(
                                evaluateVelocity(
                                    segment,
                                    joint,
                                    h)));

                    const double d =
                        segment.d[joint];

                    const double c =
                        segment.c[joint];

                    if (std::abs(d) >
                        1e-18)
                    {
                        const double tau =
                            -c /
                            (3.0 * d);

                        if (tau > 0.0 &&
                            tau < h)
                        {
                            peak_velocity =
                                std::max(
                                    peak_velocity,
                                    std::abs(
                                        evaluateVelocity(
                                            segment,
                                            joint,
                                            tau)));
                        }
                    }

                    /*
                     * 加速度：
                     *
                     * q_ddot(tau)
                     *     =
                     * 2c
                     * + 6d*tau
                     *
                     * 它在 segment 内是线性的，
                     * 所以绝对最大值一定出现在两端之一。
                     */
                    const double peak_acceleration =
                        std::max(
                            std::abs(
                                evaluateAcceleration(
                                    segment,
                                    joint,
                                    0.0)),
                            std::abs(
                                evaluateAcceleration(
                                    segment,
                                    joint,
                                    h)));

                    const double velocity_ratio =
                        peak_velocity /
                        allowed_velocity;

                    const double acceleration_ratio =
                        peak_acceleration /
                        allowed_acceleration;

                    if (velocity_ratio >
                        result.velocity_ratio)
                    {
                        result.velocity_ratio =
                            velocity_ratio;

                        result.velocity_joint =
                            joint_names_[joint];
                    }

                    if (acceleration_ratio >
                        result.acceleration_ratio)
                    {
                        result.acceleration_ratio =
                            acceleration_ratio;

                        result.acceleration_joint =
                            joint_names_[joint];
                    }
                }
            }

            return result;
        }

        /**
         * 检查连续 cubic spline 是否在两个合法 waypoint 中间
         * overshoot 出关节位置范围。
         */
        void validateContinuousPositionBounds(
            const JointPositionConstraints<N>
                &constraints) const
        {
            const auto &bounds =
                constraints.bounds();

            for (const auto &segment :
                 segments_)
            {
                for (std::size_t joint = 0;
                     joint < N;
                     ++joint)
                {
                    if (!bounds[joint])
                    {
                        continue;
                    }

                    const double lower =
                        bounds[joint]->lower();

                    const double upper =
                        bounds[joint]->upper();

                    const auto check_position =
                        [&](const double tau)
                    {
                        const double q =
                            evaluatePosition(
                                segment,
                                joint,
                                tau);

                        if (!std::isfinite(q) ||
                            q <
                                lower - 1e-12 ||
                            q >
                                upper + 1e-12)
                        {
                            throw std::runtime_error(
                                "Cubic trajectory overshoots "
                                "joint position bound: " +
                                joint_names_[joint]);
                        }
                    };

                    /*
                     * 先检查 segment 两端。
                     */
                    check_position(
                        0.0);

                    check_position(
                        segment.duration);

                    /*
                     * 中间位置极值满足：
                     *
                     *     q_dot(tau) = 0
                     *
                     * 也就是一个二次方程。
                     */
                    const double A =
                        3.0 *
                        segment.d[joint];

                    const double B =
                        2.0 *
                        segment.c[joint];

                    const double C =
                        segment.b[joint];

                    const double scale =
                        std::max(
                            {1.0,
                             std::abs(A),
                             std::abs(B),
                             std::abs(C)});

                    /*
                     * 如果二次项几乎为 0，
                     * 退化成一次方程。
                     */
                    if (std::abs(A) <=
                        1e-14 * scale)
                    {
                        if (std::abs(B) >
                            1e-14 * scale)
                        {
                            const double root =
                                -C / B;

                            if (root > 0.0 &&
                                root <
                                    segment.duration)
                            {
                                check_position(
                                    root);
                            }
                        }

                        continue;
                    }

                    double discriminant =
                        B * B -
                        4.0 * A * C;

                    /*
                     * 浮点误差可能得到非常小的负数。
                     */
                    if (discriminant < 0.0 &&
                        discriminant >
                            -1e-14 *
                                scale *
                                scale)
                    {
                        discriminant =
                            0.0;
                    }

                    if (discriminant < 0.0)
                    {
                        continue;
                    }

                    const double root_value =
                        std::sqrt(
                            discriminant);

                    const double root1 =
                        (-B + root_value) /
                        (2.0 * A);

                    const double root2 =
                        (-B - root_value) /
                        (2.0 * A);

                    if (root1 > 0.0 &&
                        root1 <
                            segment.duration)
                    {
                        check_position(
                            root1);
                    }

                    if (root2 > 0.0 &&
                        root2 <
                            segment.duration &&
                        std::abs(
                            root2 -
                            root1) >
                            1e-12)
                    {
                        check_position(
                            root2);
                    }
                }
            }
        }

        /**
         * 计算某一个 segment 在局部时间 tau 时：
         *
         *     所有关节的位置
         *     所有关节的速度
         *     所有关节的加速度
         */
        Point evaluateSegment(
            const Segment &segment,
            const double tau,
            const bool finished) const
        {
            Point point;

            for (std::size_t joint = 0;
                 joint < N;
                 ++joint)
            {
                point.position[joint] =
                    evaluatePosition(
                        segment,
                        joint,
                        tau);

                point.velocity[joint] =
                    evaluateVelocity(
                        segment,
                        joint,
                        tau);

                point.acceleration[joint] =
                    evaluateAcceleration(
                        segment,
                        joint,
                        tau);
            }

            point.finished =
                finished;

            return point;
        }

        /**
         * q(tau)
         */
        static double evaluatePosition(
            const Segment &segment,
            const std::size_t joint,
            const double tau)
        {
            return segment.a[joint] +
                   segment.b[joint] *
                       tau +
                   segment.c[joint] *
                       tau *
                       tau +
                   segment.d[joint] *
                       tau *
                       tau *
                       tau;
        }

        /**
         * q_dot(tau)
         */
        static double evaluateVelocity(
            const Segment &segment,
            const std::size_t joint,
            const double tau)
        {
            return segment.b[joint] +
                   2.0 *
                       segment.c[joint] *
                       tau +
                   3.0 *
                       segment.d[joint] *
                       tau *
                       tau;
        }

        /**
         * q_ddot(tau)
         */
        static double evaluateAcceleration(
            const Segment &segment,
            const std::size_t joint,
            const double tau)
        {
            return 2.0 *
                       segment.c[joint] +
                   6.0 *
                       segment.d[joint] *
                       tau;
        }

    private:
        CubicJointTrajectoryOptions
            options_{};

        /*
         * 最终时间缩放后的 waypoint 时间。
         */
        std::vector<double>
            times_;

        /*
         * 原始关节 waypoint。
         */
        std::vector<JointVector>
            positions_;

        /*
         * 连续 cubic segment。
         */
        std::vector<Segment>
            segments_;

        /*
         * q 顺序下的动态限制。
         */
        std::array<double, N>
            velocity_limits_{};

        std::array<double, N>
            acceleration_limits_{};

        std::array<std::string, N>
            joint_names_{};

        /*
         * 最终轨迹总时间。
         */
        double total_seconds_{0.0};

        Duration duration_{};

        /*
         * 相对于输入 VelocityTimedJointPath
         * 整体放慢的倍率。
         */
        double time_scale_{1.0};

        /*
         * 最终轨迹动态限制使用率。
         */
        double max_velocity_ratio_{0.0};

        double max_acceleration_ratio_{0.0};

        std::string
            limiting_velocity_joint_;

        std::string
            limiting_acceleration_joint_;
    };

} // namespace robot::motion::planning