#pragma once

#include "motion/trajectory/joint_trajectory.hpp"
#include "motion/trajectory/quintic_joint_trajectory.hpp"

#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot::motion
{

    // ============================================================
    // PiecewiseQuinticJointTrajectory
    // ============================================================
    //
    // 多段五次关节轨迹。
    //
    // 单段 Quintic 是：
    //
    //     q_start → q_goal
    //
    // Piecewise Quintic 是：
    //
    //     q0 → q1 → q2 → ... → qN
    //
    // 相邻两个 waypoint 之间建立一个
    // QuinticJointTrajectory。
    //
    // ------------------------------------------------------------
    //
    // 典型来源:
    //
    // Cartesian linear motion:
    //
    //     Pose0
    //       ↓ IK
    //      q0
    //
    //     Pose1
    //       ↓ IK
    //      q1
    //
    //     Pose2
    //       ↓ IK
    //      q2
    //
    // 最终:
    //
    //     q0 → q1 → q2
    //
    // ------------------------------------------------------------
    //
    // Motion Planner 也可能生成:
    //
    //     q0
    //      ↓
    //     q1
    //      ↓
    //     q2
    //      ↓
    //     q3
    //
    // 然后经过时间参数化，形成 timed waypoints。
    // 本类负责把这些 waypoint 组织成可执行轨迹。
    //
    // ------------------------------------------------------------
    //
    // 本类不负责:
    //
    //     - 生成 waypoint
    //     - IK
    //     - Cartesian path
    //     - 碰撞检测
    //     - 自动时间参数化
    //
    // 它只执行已经给定时间的关节 waypoint。
    //
    // ============================================================

    template <std::size_t N> class PiecewiseQuinticJointTrajectory final : public JointTrajectory<N>
    {
    public:
        using Base = JointTrajectory<N>;

        using Duration = typename Base::Duration;

        using Point = typename Base::Point;

        using JointValues = std::array<double, N>;

        // ========================================================
        // Waypoint
        // ========================================================
        //
        // 一个带有时间信息的关节状态。
        //
        // time_from_start:
        //
        //     这个 waypoint 相对于整条轨迹起点的时间。
        //
        // 例如：
        //
        //     waypoint 0 : 0.0 s
        //     waypoint 1 : 0.5 s
        //     waypoint 2 : 1.2 s
        //     waypoint 3 : 2.0 s
        //
        // position / velocity / acceleration
        // 用于定义该 waypoint 的完整边界状态。
        //
        // 如果相邻 segment 在 waypoint 处使用相同的
        // velocity 和 acceleration，
        // 可以实现更好的连续性。
        struct Waypoint
        {
            Duration time_from_start{};

            JointValues position{};
            JointValues velocity{};
            JointValues acceleration{};
        };

    public:
        explicit PiecewiseQuinticJointTrajectory(const std::vector<Waypoint> &waypoints)
        {
            if (waypoints.size() < 2)
            {
                throw std::invalid_argument("Piecewise quintic trajectory requires at least two waypoints");
            }

            // 为了让 sample(elapsed) 的时间定义简单明确，
            // 第一 waypoint 必须对应 trajectory 起始时刻。
            if (waypoints.front().time_from_start != Duration::zero())
            {
                throw std::invalid_argument("First waypoint time must be zero");
            }

            segments_.reserve(waypoints.size() - 1);

            for (std::size_t i = 0; i + 1 < waypoints.size(); ++i)
            {
                const auto &start = waypoints[i];

                const auto &goal = waypoints[i + 1];

                // waypoint 时间必须严格递增。
                //
                // 不允许：
                //
                //     t1 == t0
                //
                // 或：
                //
                //     t1 < t0
                //
                // 否则 segment duration 无意义。
                if (goal.time_from_start <= start.time_from_start)
                {
                    throw std::invalid_argument("Waypoint times must be strictly increasing");
                }

                typename QuinticJointTrajectory<N>::BoundaryState start_state{start.position, start.velocity, start.acceleration};

                typename QuinticJointTrajectory<N>::BoundaryState goal_state{goal.position, goal.velocity, goal.acceleration};

                const Duration segment_duration = goal.time_from_start - start.time_from_start;

                segments_.push_back(Segment{start.time_from_start, goal.time_from_start, QuinticJointTrajectory<N>(start_state, goal_state, segment_duration)});
            }

            duration_ = waypoints.back().time_from_start;
        }

    public:
        Point sample(Duration elapsed) const override
        {
            // ----------------------------------------------------
            // Before start
            // ----------------------------------------------------

            if (elapsed <= Duration::zero())
            {
                return segments_.front().trajectory.sample(Duration::zero());
            }

            // ----------------------------------------------------
            // After trajectory end
            // ----------------------------------------------------

            if (elapsed >= duration_)
            {
                return segments_.back().trajectory.sample(segments_.back().trajectory.duration());
            }

            // ----------------------------------------------------
            // Find active segment
            // ----------------------------------------------------
            //
            // 第一版使用简单线性搜索。
            //
            // waypoint 数量通常不会非常大，
            // 而且代码最容易理解。
            //
            // 以后如果 waypoint 数量很大，
            // 可以再改成 binary search。
            for (const auto &segment : segments_)
            {
                if (elapsed < segment.end_time)
                {
                    const Duration local_elapsed = elapsed - segment.start_time;

                    auto point = segment.trajectory.sample(local_elapsed);

                    // 整条 Piecewise trajectory
                    // 还没有结束。
                    //
                    // 即使某个内部 segment 已经接近结束，
                    // finished 仍然应该表示整条 trajectory。
                    point.finished = false;

                    return point;
                }
            }

            // 理论上不会进入这里，
            // 仅作为安全 fallback。
            return segments_.back().trajectory.sample(segments_.back().trajectory.duration());
        }

        Duration duration() const noexcept override
        {
            return duration_;
        }

    private:
        // ========================================================
        // Segment
        // ========================================================
        //
        // 整条 Piecewise trajectory 中的一个时间段。
        //
        // global time:
        //
        //     start_time -------- end_time
        //
        // segment 内部 trajectory 使用 local time:
        //
        //     0 -------- segment_duration
        struct Segment
        {
            Duration start_time{};
            Duration end_time{};

            QuinticJointTrajectory<N> trajectory;
        };

    private:
        std::vector<Segment> segments_;

        Duration duration_{};
    };

} // namespace robot::motion