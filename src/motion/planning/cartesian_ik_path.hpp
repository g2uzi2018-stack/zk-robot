#pragma once

#include "motion/kinematics/numerical_ik.hpp"
#include "motion/path/cartesian_line_path.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot::motion::planning
{

    /**
     * 笛卡尔几何路径经过连续 IK 后的停止原因。
     *
     * Completed：
     *     整条 CartesianLinePath 的所有采样点均成功得到合法关节解。
     *
     * 其余状态都表示没有得到完整路径。
     * 失败时 result.waypoints 仍可保存已经成功求出的前缀，
     * 但这个前缀不能被当作完整 MoveL 自动执行。
     */
    enum class CartesianIkPathStatus
    {
        Completed,
        InvalidInput,
        StartOutOfBounds,
        StartPoseMismatch,
        SamplingLimit,
        IkFailure,
        JointJump,
        TimeLimit,
        NumericalFailure
    };

    /**
     * 笛卡尔路径上的一个离散关节 waypoint。
     *
     * progress：
     *     对应 CartesianLinePath 的 [0, 1] 进度。
     *
     * target_pose：
     *     这个 progress 对应的笛卡尔目标 Pose。
     *
     * q：
     *     IK 求出的关节位置。
     *
     * ik_iterations / position_error / rotation_error：
     *     这一采样点的 IK 诊断信息。
     *
     * 第 0 个 waypoint 不需要运行 IK：
     *     q 就是 start_q，
     *     但会检查 FK(start_q) 是否与 path.start() 一致。
     */
    template <std::size_t N>
    struct CartesianIkWaypoint
    {
        double progress{0.0};
        Pose target_pose{};
        std::array<double, N> q{};

        std::size_t ik_iterations{0};
        double position_error{0.0};
        double rotation_error{0.0};
    };

    /**
     * 连续笛卡尔 IK 的路径级参数。
     *
     * max_translation_step：
     *     相邻笛卡尔采样点最大平移距离，单位 m。
     *
     * max_rotation_step：
     *     相邻笛卡尔采样点最大姿态变化角，单位 rad。
     *
     * 最终段数同时满足平移和姿态两个采样要求：
     *
     *     segments >= translation_length / max_translation_step
     *     segments >= rotation_angle     / max_rotation_step
     *
     * max_rotational_joint_step / max_prismatic_joint_step：
     *     两个相邻已经接受的 IK waypoint 之间，
     *     单个关节允许的最大位置跳变量。
     *
     * 注意：
     *     这是“路径 waypoint 之间的连续性检查”，
     *     不是 numerical IK 内部单次迭代的 max_*_step。
     *
     * start_*_tolerance：
     *     检查 start_q 做 FK 后是否真的对应 path.start()。
     *
     * max_segments：
     *     限制笛卡尔离散点数量，避免异常参数制造无限工作量。
     *
     * total_time_budget：
     *     整条路径规划允许的总计算时间。
     *     不是机器人运行这条路径所需要的运动时间。
     *
     * 这里的默认值只是初始工程参数，不是真机精度或性能承诺。
     */
    struct CartesianIkPathOptions
    {
        double max_translation_step{0.005}; // 5 mm
        double max_rotation_step{0.05};     // rad

        double max_rotational_joint_step{0.35}; // rad / waypoint
        double max_prismatic_joint_step{0.03};  // m / waypoint

        double start_position_tolerance{1e-4}; // m
        double start_rotation_tolerance{1e-3}; // rad

        std::size_t max_segments{2000};

        std::chrono::duration<double> total_time_budget{2.0};
    };

    /**
     * 整条连续 IK 路径的结果。
     *
     * waypoints：
     *     包含起点以及所有成功接受的后续点。
     *
     * completed_progress：
     *     最后一个成功 waypoint 的 progress。
     *     Completed 时一定为 1.0。
     *
     * failed_progress：
     *     如果在某个笛卡尔采样点失败，记录该点的 progress。
     *
     * failed_ik_status：
     *     只有 CartesianIkPathStatus::IkFailure 时有意义，
     *     保存底层单点 IK 的停止原因。
     *
     * max_observed_*_joint_step：
     *     实际成功路径中观察到的最大相邻关节跳变量，
     *     供调试和后续参数选择使用。
     */
    template <std::size_t N>
    struct CartesianIkPathResult
    {
        CartesianIkPathStatus status{CartesianIkPathStatus::InvalidInput};

        std::vector<CartesianIkWaypoint<N>> waypoints;

        double completed_progress{0.0};
        std::optional<double> failed_progress;
        std::optional<IkStatus> failed_ik_status;

        double max_observed_rotational_joint_step{0.0};
        double max_observed_prismatic_joint_step{0.0};

        std::size_t segment_count{0};

        std::chrono::duration<double> elapsed{};
        std::string message;

        [[nodiscard]] bool completed() const noexcept
        {
            return status == CartesianIkPathStatus::Completed;
        }
    };

    namespace detail
    {

        using CartesianIkClock = std::chrono::steady_clock;

        /** 只验证路径层自己的参数；单点 IK 参数仍由 solveNumericalIk() 验证。 */
        inline bool validCartesianIkPathOptions(
            const CartesianIkPathOptions &options)
        {
            const double values[]{
                options.max_translation_step,
                options.max_rotation_step,
                options.max_rotational_joint_step,
                options.max_prismatic_joint_step,
                options.start_position_tolerance,
                options.start_rotation_tolerance,
                options.total_time_budget.count()};

            for (const double value : values)
            {
                if (!std::isfinite(value) || value <= 0.0)
                {
                    return false;
                }
            }

            return options.max_segments > 0;
        }

    } // namespace detail

    /**
     * 将一条 CartesianLinePath 离散化，并逐点做连续 IK。
     *
     * 上游：
     *     model / constraints：
     *         初始化阶段准备好的运动学模型与有效关节范围。
     *
     *     path：
     *         已经定义好的笛卡尔直线路径。
     *
     *     start_q：
     *         机器人规划起点的完整关节位置，不是增量。
     *
     *     ik_options：
     *         每个单点 IK 使用的参数。
     *
     * 工作机制：
     *
     *   1. FK(start_q)，确认它对应 path.start()。
     *
     *   2. 根据：
     *          路径平移长度
     *          路径姿态旋转角
     *      计算离散段数。
     *
     *   3. progress = 1/segments：
     *          target = path.sample(progress)
     *          seed   = start_q
     *          solveNumericalIk(...)
     *
     *   4. progress = 2/segments：
     *          target = path.sample(progress)
     *          seed   = 上一点求出的 q
     *
     *   5. 继续直到 progress = 1。
     *
     * 所以：
     *
     *     q_0 --seed--> target_1 --IK--> q_1
     *     q_1 --seed--> target_2 --IK--> q_2
     *     q_2 --seed--> target_3 --IK--> q_3
     *     ...
     *
     * 这种做法利用相邻笛卡尔点通常比较接近的特点，
     * 让 IK 尽量沿当前关节解分支连续前进。
     *
     * 但“上一点做 seed”不能数学上保证不会切换到另一组 IK 分支，
     * 因此每次成功后还检查相邻 q 的跳变量。
     *
     * 本函数不做：
     *     - 碰撞检测；
     *     - 速度/加速度/jerk 时间参数化；
     *     - 电机控制；
     *     - 多 seed 全局搜索；
     *     - 失败点自动改变笛卡尔几何路径。
     *
     * 输出仍然只是离散关节路径，不是最终 JointTrajectory。
     */
    template <std::size_t N>
    [[nodiscard]] CartesianIkPathResult<N> solveCartesianLineIk(
        const ValidatedUrdfChain<N> &model,
        const JointPositionConstraints<N> &constraints,
        const CartesianLinePath &path,
        const std::array<double, N> &start_q,
        const CartesianIkPathOptions &path_options = {},
        const IkOptions &ik_options = {})
    {
        static_assert(N > 0, "Cartesian IK path requires active joints");

        using namespace detail;

        const auto start_time = CartesianIkClock::now();

        CartesianIkPathResult<N> result;

        const auto finish =
            [&](const CartesianIkPathStatus status,
                const std::string &message)
        {
            result.status = status;
            result.message = message;
            result.elapsed =
                std::chrono::duration<double>(
                    CartesianIkClock::now() - start_time);

            return result;
        };

        if (!validCartesianIkPathOptions(path_options))
        {
            return finish(
                CartesianIkPathStatus::InvalidInput,
                "Invalid Cartesian IK path options");
        }

        try
        {
            constraints.requireJointOrder(model);

            for (const double value : start_q)
            {
                if (!std::isfinite(value))
                {
                    return finish(
                        CartesianIkPathStatus::InvalidInput,
                        "Non-finite Cartesian path start_q");
                }
            }

            if (!constraints.contains(start_q))
            {
                return finish(
                    CartesianIkPathStatus::StartOutOfBounds,
                    "Cartesian path start_q violates position bounds");
            }

            /*
             * 先确认：
             *
             *     start_q --FK--> 实际起点
             *
             * 与调用者交给 CartesianLinePath 的 start 是一致的。
             *
             * 否则后面虽然会沿 path.start() -> path.goal() 求解，
             * 但实际机器人关节起点却不在这条几何路径的起点上。
             */
            const Pose actual_start =
                forwardKinematics(model, start_q);

            const Eigen::Matrix<double, 6, 1> start_error =
                poseErrorInBase(actual_start, path.start());

            const double start_position_error =
                start_error.head<3>().stableNorm();

            const double start_rotation_error =
                start_error.tail<3>().stableNorm();

            if (start_position_error >
                    path_options.start_position_tolerance ||
                start_rotation_error >
                    path_options.start_rotation_tolerance)
            {
                return finish(
                    CartesianIkPathStatus::StartPoseMismatch,
                    "start_q FK does not match Cartesian path start pose");
            }

            /*
             * 计算笛卡尔采样段数。
             *
             * 例如：
             *
             *     translation = 0.10 m
             *     max_translation_step = 0.005 m
             *
             * 至少需要：
             *
             *     ceil(0.10 / 0.005) = 20 段
             *
             * 姿态同理。
             *
             * 最后取两者较大值，使两个采样约束同时满足。
             */
            const double translation_length =
                path.translationLength();

            const double rotation_angle =
                path.rotationAngle();

            if (!std::isfinite(translation_length) ||
                !std::isfinite(rotation_angle))
            {
                return finish(
                    CartesianIkPathStatus::NumericalFailure,
                    "Non-finite Cartesian path length");
            }

            const double translation_segments =
                std::ceil(
                    translation_length /
                    path_options.max_translation_step);

            const double rotation_segments =
                std::ceil(
                    rotation_angle /
                    path_options.max_rotation_step);

            const double required_segments =
                std::max(
                    translation_segments,
                    rotation_segments);

            if (!std::isfinite(required_segments) ||
                required_segments >
                    static_cast<double>(
                        path_options.max_segments))
            {
                return finish(
                    CartesianIkPathStatus::SamplingLimit,
                    "Cartesian path requires too many segments");
            }

            result.segment_count =
                static_cast<std::size_t>(
                    required_segments);

            /*
             * 保存 q 下标对应的关节类型。
             *
             * 后面检查相邻 waypoint 的关节跳变时，
             * Revolute / Continuous 使用 rad 阈值，
             * Prismatic 使用 m 阈值。
             */
            std::array<UrdfJointType, N> joint_types{};

            for (const auto &joint : model.chain().joints)
            {
                if (!joint.q_index.has_value())
                {
                    continue;
                }

                joint_types[*joint.q_index] =
                    joint.type;
            }

            /*
             * 第一个 waypoint 就是实际起点。
             *
             * 它不需要再执行一次 IK，因为 start_q 已经存在，
             * 而且上面已经验证 FK(start_q) 与 path.start() 一致。
             */
            CartesianIkWaypoint<N> start_waypoint;
            start_waypoint.progress = 0.0;
            start_waypoint.target_pose = path.start();
            start_waypoint.q = start_q;
            start_waypoint.position_error =
                start_position_error;
            start_waypoint.rotation_error =
                start_rotation_error;

            result.waypoints.push_back(
                start_waypoint);

            result.completed_progress = 0.0;

            /*
             * 零长度、零旋转路径：
             *
             * 没有后续笛卡尔采样点。
             * start_q 已经通过起点一致性检查，
             * 因此直接得到一条只有起点的完成路径。
             */
            if (result.segment_count == 0)
            {
                result.completed_progress = 1.0;

                return finish(
                    CartesianIkPathStatus::Completed,
                    "Cartesian path has zero length");
            }

            std::array<double, N> current_q =
                start_q;

            /*
             * progress 从第 1 段开始。
             *
             * 每一个成功的 solved_q 都会成为下一点的 seed。
             */
            for (std::size_t segment = 1;
                 segment <= result.segment_count;
                 ++segment)
            {
                const auto now =
                    CartesianIkClock::now();

                const auto elapsed =
                    std::chrono::duration<double>(
                        now - start_time);

                const auto remaining =
                    path_options.total_time_budget -
                    elapsed;

                if (remaining.count() <= 0.0)
                {
                    result.failed_progress =
                        result.completed_progress;

                    return finish(
                        CartesianIkPathStatus::TimeLimit,
                        "Cartesian IK path time budget exhausted");
                }

                /*
                 * 最后一段显式使用 1.0，
                 * 避免整数转浮点除法产生 0.999999... 之类的终点误差。
                 */
                const double progress =
                    (segment == result.segment_count)
                        ? 1.0
                        : static_cast<double>(segment) /
                              static_cast<double>(
                                  result.segment_count);

                const Pose target =
                    path.sample(progress);

                /*
                 * 单点 IK 的预算不能超过整条路径剩余预算。
                 *
                 * 如果调用者给每点 0.2 s，
                 * 但整条路径只剩 0.03 s，
                 * 这一点最多只能使用剩余的 0.03 s。
                 */
                IkOptions local_ik_options =
                    ik_options;

                local_ik_options.time_budget =
                    std::min(
                        ik_options.time_budget,
                        remaining);

                const IkResult<N> ik_result =
                    solveNumericalIk(
                        model,
                        constraints,
                        target,
                        current_q,
                        local_ik_options);

                if (!ik_result.converged() ||
                    !ik_result.candidate_q.has_value())
                {
                    result.failed_progress =
                        progress;

                    result.failed_ik_status =
                        ik_result.status;

                    return finish(
                        CartesianIkPathStatus::IkFailure,
                        "IK failed at Cartesian progress " +
                            std::to_string(progress) +
                            ": " +
                            ik_result.message);
                }

                const std::array<double, N> solved_q =
                    *ik_result.candidate_q;

                /*
                 * IK 成功不等于关节路径一定连续。
                 *
                 * 例如冗余机械臂可能存在两套不同构型，
                 * 两套构型都能到达几乎相同的末端 Pose。
                 *
                 * 如果相邻采样点突然从一套构型跳到另一套，
                 * FK 仍然可能正确，但这个路径不能直接当作连续 MoveL。
                 */
                for (std::size_t i = 0; i < N; ++i)
                {
                    const double delta =
                        std::abs(
                            solved_q[i] -
                            current_q[i]);

                    if (!std::isfinite(delta))
                    {
                        return finish(
                            CartesianIkPathStatus::NumericalFailure,
                            "Non-finite joint delta in Cartesian IK path");
                    }

                    const bool prismatic =
                        joint_types[i] ==
                        UrdfJointType::Prismatic;

                    const double allowed =
                        prismatic
                            ? path_options
                                  .max_prismatic_joint_step
                            : path_options
                                  .max_rotational_joint_step;

                    if (prismatic)
                    {
                        result.max_observed_prismatic_joint_step =
                            std::max(
                                result.max_observed_prismatic_joint_step,
                                delta);
                    }
                    else
                    {
                        result.max_observed_rotational_joint_step =
                            std::max(
                                result.max_observed_rotational_joint_step,
                                delta);
                    }

                    if (delta > allowed)
                    {
                        result.failed_progress =
                            progress;

                        return finish(
                            CartesianIkPathStatus::JointJump,
                            "Joint jump at " +
                                constraints.jointNames()[i] +
                                ", progress=" +
                                std::to_string(progress));
                    }
                }

                /*
                 * 到这里：
                 *
                 *   该 Pose 的 IK 已成功；
                 *   solved_q 满足位置约束；
                 *   与上一点之间也没有超过关节跳变阈值。
                 *
                 * 才正式接受这个 waypoint。
                 */
                CartesianIkWaypoint<N> waypoint;
                waypoint.progress = progress;
                waypoint.target_pose = target;
                waypoint.q = solved_q;
                waypoint.ik_iterations =
                    ik_result.iterations;
                waypoint.position_error =
                    ik_result.position_error;
                waypoint.rotation_error =
                    ik_result.rotation_error;

                result.waypoints.push_back(
                    waypoint);

                current_q = solved_q;

                result.completed_progress =
                    progress;
            }

            /*
             * 能走到这里说明所有采样点都完成。
             */
            result.completed_progress = 1.0;

            return finish(
                CartesianIkPathStatus::Completed,
                "Cartesian IK path completed");
        }
        catch (const std::invalid_argument &error)
        {
            return finish(
                CartesianIkPathStatus::InvalidInput,
                error.what());
        }
        catch (const std::runtime_error &error)
        {
            return finish(
                CartesianIkPathStatus::NumericalFailure,
                error.what());
        }
        catch (const std::logic_error &error)
        {
            return finish(
                CartesianIkPathStatus::NumericalFailure,
                error.what());
        }
    }

} // namespace robot::motion::planning