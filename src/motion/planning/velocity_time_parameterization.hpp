#pragma once

#include "motion/kinematics/validated_urdf_chain.hpp"
#include "motion/planning/cartesian_ik_path.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motion::planning
{

    /**
     * 上游按关节名称提供最大速度。
     *
     * Revolute / Continuous：
     *     单位 rad/s
     *
     * Prismatic：
     *     单位 m/s
     *
     * 这里只保存“关节侧”的真实速度限制，
     * 不保存电机 rpm，也不负责减速比换算。
     *
     * 对 TIAGO：
     *     可以从 tiago::JointConfig::limits.max_velocity 转换进来。
     *
     * motion 层不直接 include tiago/can/can_config.hpp，
     * 这样规划算法仍然可以用于别的机器人。
     */
    struct NamedJointVelocityLimit
    {
        std::string joint_name;
        double max_velocity{0.0};
    };

    /**
     * 把按名称给出的速度限制整理成运动学 q 顺序：
     *
     *     q[0] -> max_velocity[0]
     *     q[1] -> max_velocity[1]
     *     ...
     *
     * 这与 JointPositionConstraints 的思想相同：
     * 上游配置可以按任意顺序提供，
     * 进入规划器前统一转换为 q 顺序。
     *
     * 初始化时严格要求：
     *     - 每个活动关节必须且只能出现一次；
     *     - 不允许未知关节；
     *     - max_velocity 必须有限且 > 0；
     *     - 固定关节不需要速度限制。
     */
    template <std::size_t N>
    class JointVelocityLimits final
    {
    public:
        using VelocityArray = std::array<double, N>;
        using NameArray = std::array<std::string, N>;

        explicit JointVelocityLimits(
            const ValidatedUrdfChain<N> &model,
            const std::vector<NamedJointVelocityLimit> &configured_limits)
        {
            std::unordered_map<std::string, double> by_name;

            for (const auto &configured : configured_limits)
            {
                if (configured.joint_name.empty())
                {
                    throw std::invalid_argument(
                        "Velocity-limit joint name must not be empty");
                }

                if (!std::isfinite(configured.max_velocity) ||
                    configured.max_velocity <= 0.0)
                {
                    throw std::invalid_argument(
                        "Joint max velocity must be finite and positive: " +
                        configured.joint_name);
                }

                if (!by_name
                         .emplace(
                             configured.joint_name,
                             configured.max_velocity)
                         .second)
                {
                    throw std::invalid_argument(
                        "Duplicate joint velocity limit: " +
                        configured.joint_name);
                }
            }

            std::array<bool, N> filled{};

            for (const auto &joint : model.chain().joints)
            {
                if (!joint.q_index.has_value())
                {
                    continue;
                }

                const std::size_t q_index =
                    *joint.q_index;

                if (q_index >= N)
                {
                    throw std::logic_error(
                        "Joint q_index exceeds velocity-limit dimension");
                }

                const auto configured =
                    by_name.find(joint.name);

                if (configured == by_name.end())
                {
                    throw std::invalid_argument(
                        "Missing velocity limit for active joint: " +
                        joint.name);
                }

                joint_names_[q_index] =
                    joint.name;

                max_velocity_[q_index] =
                    configured->second;

                filled[q_index] = true;

                by_name.erase(configured);
            }

            for (std::size_t i = 0; i < N; ++i)
            {
                if (!filled[i])
                {
                    throw std::logic_error(
                        "Incomplete q ordering while building velocity limits");
                }
            }

            /*
             * 如果还有剩余项，
             * 说明配置中出现了当前运动链不认识的关节名称。
             *
             * 不静默忽略，避免名字写错后规划器仍继续运行。
             */
            if (!by_name.empty())
            {
                throw std::invalid_argument(
                    "Velocity limit provided for unknown active joint: " +
                    by_name.begin()->first);
            }
        }

        const VelocityArray &maxVelocity() const noexcept
        {
            return max_velocity_;
        }

        const NameArray &jointNames() const noexcept
        {
            return joint_names_;
        }

        /**
         * 在真正规划前确认：
         * 当前 limits 的 q 顺序仍然对应当前 model。
         *
         * 防止把另一个机器人、另一条手臂或另一套关节排序
         * 的限制对象错误复用到这里。
         */
        void requireJointOrder(
            const ValidatedUrdfChain<N> &model) const
        {
            std::size_t active_index = 0;

            for (const auto &joint : model.chain().joints)
            {
                if (!joint.q_index.has_value())
                {
                    continue;
                }

                if (active_index >= N ||
                    *joint.q_index != active_index ||
                    joint_names_[active_index] != joint.name)
                {
                    throw std::invalid_argument(
                        "Joint velocity limits do not match model q order");
                }

                ++active_index;
            }

            if (active_index != N)
            {
                throw std::invalid_argument(
                    "Joint velocity-limit dimension does not match model");
            }
        }

    private:
        NameArray joint_names_{};
        VelocityArray max_velocity_{};
    };

    /**
     * 速度时间参数化的参数。
     *
     * velocity_scaling：
     *
     *     1.0  -> 最多使用配置最大速度的 100%
     *     0.5  -> 最多使用配置最大速度的 50%
     *
     * 必须位于 (0, 1]。
     *
     * minimum_segment_duration：
     *
     *     即使某一段 q 变化极小，也至少给这一段这么多时间。
     *
     *     主要用于避免产生极小时间间隔，
     *     也给后续加速度时间参数化保留合理的数值尺度。
     *
     * 这不是控制周期。
     */
    struct VelocityTimeParameterizationOptions
    {
        double velocity_scaling{1.0};

        std::chrono::duration<double>
            minimum_segment_duration{1e-4};
    };

    enum class VelocityTimeParameterizationStatus
    {
        Completed,
        InvalidInput,
        SourcePathIncomplete,
        NumericalFailure
    };

    /**
     * 一个已经赋予时间的关节 waypoint。
     *
     * time_from_start：
     *     从整条路径开始，到达该 waypoint 应经过的时间。
     *
     * progress：
     *     保留原 CartesianLinePath 的 progress，
     *     方便后续重新做笛卡尔路径验收。
     *
     * q：
     *     该 waypoint 的关节位置。
     *
     * 注意：
     *     这里还没有 waypoint velocity / acceleration。
     *
     * 当前文件只给离散关节点安排时间，
     * 下一阶段才会生成速度连续、加速度受限的真正轨迹。
     */
    template <std::size_t N>
    struct VelocityTimedJointWaypoint
    {
        std::chrono::duration<double>
            time_from_start{0.0};

        double progress{0.0};

        std::array<double, N> q{};
    };

    /**
     * 速度时间参数化结果。
     *
     * max_velocity_ratio：
     *
     *     实际分段速度 /
     *     (配置最大速度 * velocity_scaling)
     *
     * 理论上 Completed 时应 <= 1。
     *
     * limiting_joint_name：
     *     整条路径中最接近速度限制的关节，
     *     主要用于调试。
     */
    template <std::size_t N>
    struct VelocityTimedJointPath
    {
        VelocityTimeParameterizationStatus status{
            VelocityTimeParameterizationStatus::InvalidInput};

        std::vector<VelocityTimedJointWaypoint<N>> waypoints;

        std::chrono::duration<double>
            duration{0.0};

        double max_velocity_ratio{0.0};

        std::string limiting_joint_name;
        std::string message;

        [[nodiscard]] bool completed() const noexcept
        {
            return status ==
                   VelocityTimeParameterizationStatus::Completed;
        }
    };

    /**
     * 给已经完成连续 IK 的关节 waypoint 分配时间。
     *
     * 输入：
     *
     *     CartesianIkPathResult
     *
     *        q0
     *        q1
     *        q2
     *        ...
     *
     * 输出：
     *
     *        t0 = 0
     *        t1
     *        t2
     *        ...
     *
     * ============================================================
     * 每一段时间是怎么计算的？
     * ============================================================
     *
     * 对第 k 段：
     *
     *     q_k -> q_(k+1)
     *
     * 对每个关节 i：
     *
     *     delta_q_i =
     *         abs(q_(k+1)[i] - q_k[i])
     *
     * 如果该关节允许速度为：
     *
     *     v_i
     *
     * 那么仅从这个关节来看，
     * 至少需要：
     *
     *     dt_i = delta_q_i / v_i
     *
     * 秒。
     *
     * 但是所有关节必须同步从 waypoint k 到 waypoint k+1，
     * 因此真正的 segment duration 取所有关节中最大的那个：
     *
     *     dt_segment = max(dt_0, dt_1, ... dt_N)
     *
     * 这样：
     *
     *     变化最大的 / 速度最慢的那个关节
     *
     * 决定这一段至少要花多少时间。
     *
     * 其他关节因为使用相同 dt_segment，
     * 实际速度只会更低，不会超过限制。
     *
     * ============================================================
     * 举例
     * ============================================================
     *
     * 两个关节：
     *
     *     joint0:
     *         delta = 0.10 rad
     *         vmax  = 1.0 rad/s
     *
     *         至少需要 0.10 s
     *
     *     joint1:
     *         delta = 0.20 rad
     *         vmax  = 1.0 rad/s
     *
     *         至少需要 0.20 s
     *
     * 两个关节必须一起从这个 waypoint 到下一个 waypoint，
     * 所以整段：
     *
     *     dt = 0.20 s
     *
     * 实际速度：
     *
     *     joint0 = 0.10 / 0.20 = 0.5 rad/s
     *     joint1 = 0.20 / 0.20 = 1.0 rad/s
     *
     * 都不超限。
     *
     * ============================================================
     * 非常重要
     * ============================================================
     *
     * 当前算法保证的是：
     *
     *     如果两个 waypoint 之间按关节位置线性运动，
     *     那么该段平均/常值关节速度不会超过最大速度。
     *
     * 它还没有解决 waypoint 边界的速度跳变：
     *
     *     segment 0 velocity
     *              ↓
     *           waypoint
     *              ↓
     *     segment 1 velocity
     *
     * 两边速度可能不同。
     *
     * 如果直接瞬时切换，就意味着加速度无限大。
     *
     * 因此这个结果是：
     *
     *     “速度约束下的初始时间表”
     *
     * 而不是最终可执行的工业 MoveL。
     *
     * 下一阶段需要在这个时间表基础上加入：
     *
     *     max_acceleration
     *     forward/backward adjustment
     *     或其他加速度受限时间参数化方法
     *
     * 才能形成最终轨迹。
     */
    template <std::size_t N>
    [[nodiscard]]
    VelocityTimedJointPath<N>
    parameterizeVelocityLimited(
        const ValidatedUrdfChain<N> &model,
        const JointVelocityLimits<N> &velocity_limits,
        const CartesianIkPathResult<N> &source_path,
        const VelocityTimeParameterizationOptions &options = {})
    {
        VelocityTimedJointPath<N> result;

        const auto finish =
            [&](const VelocityTimeParameterizationStatus status,
                const std::string &message)
        {
            result.status = status;
            result.message = message;
            return result;
        };

        if (!std::isfinite(options.velocity_scaling) ||
            options.velocity_scaling <= 0.0 ||
            options.velocity_scaling > 1.0 ||
            !std::isfinite(
                options.minimum_segment_duration.count()) ||
            options.minimum_segment_duration.count() <= 0.0)
        {
            return finish(
                VelocityTimeParameterizationStatus::InvalidInput,
                "Invalid velocity time-parameterization options");
        }

        try
        {
            velocity_limits.requireJointOrder(model);
        }
        catch (const std::invalid_argument &error)
        {
            return finish(
                VelocityTimeParameterizationStatus::InvalidInput,
                error.what());
        }

        /*
         * 失败的 Cartesian IK 前缀绝对不能偷偷拿来时间参数化。
         */
        if (!source_path.completed())
        {
            return finish(
                VelocityTimeParameterizationStatus::SourcePathIncomplete,
                "Cartesian IK path is incomplete");
        }

        if (source_path.waypoints.empty())
        {
            return finish(
                VelocityTimeParameterizationStatus::InvalidInput,
                "Cartesian IK path contains no waypoints");
        }

        const auto &max_velocity =
            velocity_limits.maxVelocity();

        /*
         * 验证所有输入 waypoint。
         *
         * 同时要求 progress 单调不减。
         */
        double previous_progress = -1.0;

        for (const auto &waypoint :
             source_path.waypoints)
        {
            if (!std::isfinite(waypoint.progress) ||
                waypoint.progress < 0.0 ||
                waypoint.progress > 1.0 ||
                waypoint.progress < previous_progress)
            {
                return finish(
                    VelocityTimeParameterizationStatus::InvalidInput,
                    "Invalid Cartesian waypoint progress");
            }

            for (const double q : waypoint.q)
            {
                if (!std::isfinite(q))
                {
                    return finish(
                        VelocityTimeParameterizationStatus::InvalidInput,
                        "Non-finite joint waypoint");
                }
            }

            previous_progress =
                waypoint.progress;
        }

        result.waypoints.reserve(
            source_path.waypoints.size());

        /*
         * 第一个 waypoint 的时间永远是 0。
         */
        VelocityTimedJointWaypoint<N> first;
        first.time_from_start =
            std::chrono::duration<double>{0.0};
        first.progress =
            source_path.waypoints.front().progress;
        first.q =
            source_path.waypoints.front().q;

        result.waypoints.push_back(first);

        /*
         * 只有一个 waypoint，
         * 例如零长度 Cartesian path。
         */
        if (source_path.waypoints.size() == 1)
        {
            result.duration =
                std::chrono::duration<double>{0.0};

            return finish(
                VelocityTimeParameterizationStatus::Completed,
                "Zero-motion path parameterized");
        }

        std::chrono::duration<double>
            accumulated_time{0.0};

        /*
         * 逐 segment 计算最短速度安全时间。
         */
        for (std::size_t segment = 0;
             segment + 1 < source_path.waypoints.size();
             ++segment)
        {
            const auto &from =
                source_path.waypoints[segment];

            const auto &to =
                source_path.waypoints[segment + 1];

            double segment_seconds =
                0.0;

            /*
             * 先找出所有关节各自要求的最短时间。
             */
            for (std::size_t joint = 0;
                 joint < N;
                 ++joint)
            {
                const double delta =
                    std::abs(
                        to.q[joint] -
                        from.q[joint]);

                const double allowed_velocity =
                    max_velocity[joint] *
                    options.velocity_scaling;

                if (!std::isfinite(delta) ||
                    !std::isfinite(allowed_velocity) ||
                    allowed_velocity <= 0.0)
                {
                    return finish(
                        VelocityTimeParameterizationStatus::NumericalFailure,
                        "Invalid velocity timing arithmetic");
                }

                const double required_seconds =
                    delta /
                    allowed_velocity;

                if (!std::isfinite(required_seconds))
                {
                    return finish(
                        VelocityTimeParameterizationStatus::NumericalFailure,
                        "Velocity timing overflow");
                }

                segment_seconds =
                    std::max(
                        segment_seconds,
                        required_seconds);
            }

            /*
             * 给极短运动一个正的最小时间。
             *
             * 这只会让速度更低，
             * 不会破坏速度限制。
             */
            segment_seconds =
                std::max(
                    segment_seconds,
                    options
                        .minimum_segment_duration
                        .count());

            if (!std::isfinite(segment_seconds) ||
                segment_seconds <= 0.0)
            {
                return finish(
                    VelocityTimeParameterizationStatus::NumericalFailure,
                    "Invalid segment duration");
            }

            /*
             * 用已经确定的 segment_seconds 重新算实际速度，
             * 做一次独立验收。
             */
            for (std::size_t joint = 0;
                 joint < N;
                 ++joint)
            {
                const double delta = std::abs(to.q[joint] - from.q[joint]);

                const double actual_velocity =
                    delta /
                    segment_seconds;

                const double allowed_velocity =
                    max_velocity[joint] *
                    options.velocity_scaling;

                const double ratio =
                    actual_velocity /
                    allowed_velocity;

                if (!std::isfinite(actual_velocity) ||
                    !std::isfinite(ratio))
                {
                    return finish(
                        VelocityTimeParameterizationStatus::NumericalFailure,
                        "Non-finite parameterized joint velocity");
                }

                /*
                 * 浮点计算允许极小误差。
                 */
                if (ratio > 1.0 + 1e-12)
                {
                    return finish(
                        VelocityTimeParameterizationStatus::NumericalFailure,
                        "Parameterized joint velocity exceeds limit");
                }

                if (ratio >
                    result.max_velocity_ratio)
                {
                    result.max_velocity_ratio =
                        ratio;

                    result.limiting_joint_name =
                        velocity_limits
                            .jointNames()[joint];
                }
            }

            accumulated_time +=
                std::chrono::duration<double>{
                    segment_seconds};

            if (!std::isfinite(
                    accumulated_time.count()))
            {
                return finish(
                    VelocityTimeParameterizationStatus::NumericalFailure,
                    "Trajectory duration overflow");
            }

            VelocityTimedJointWaypoint<N> timed;
            timed.time_from_start =
                accumulated_time;
            timed.progress =
                to.progress;
            timed.q =
                to.q;

            result.waypoints.push_back(
                timed);
        }

        result.duration =
            accumulated_time;

        return finish(
            VelocityTimeParameterizationStatus::Completed,
            "Velocity-limited initial timing completed");
    }

} // namespace robot::motion::planning