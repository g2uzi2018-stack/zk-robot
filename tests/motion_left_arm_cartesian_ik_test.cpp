#include "motion/kinematics/urdf_chain.hpp"
#include "motion/planning/cartesian_ik_path.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

using namespace robot::motion;
using namespace robot::motion::planning;

constexpr std::size_t kJointCount = 7;
using JointVector = std::array<double, kJointCount>;

void require(
    const bool condition,
    const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

void requireNear(
    const double actual,
    const double expected,
    const double tolerance,
    const std::string &message)
{
    if (!std::isfinite(actual) ||
        !std::isfinite(expected) ||
        std::abs(actual - expected) > tolerance)
    {
        throw std::runtime_error(
            message +
            ": actual=" + std::to_string(actual) +
            ", expected=" + std::to_string(expected));
    }
}

const char *statusName(
    const CartesianIkPathStatus status)
{
    switch (status)
    {
    case CartesianIkPathStatus::Completed:
        return "Completed";

    case CartesianIkPathStatus::InvalidInput:
        return "InvalidInput";

    case CartesianIkPathStatus::StartOutOfBounds:
        return "StartOutOfBounds";

    case CartesianIkPathStatus::StartPoseMismatch:
        return "StartPoseMismatch";

    case CartesianIkPathStatus::SamplingLimit:
        return "SamplingLimit";

    case CartesianIkPathStatus::IkFailure:
        return "IkFailure";

    case CartesianIkPathStatus::JointJump:
        return "JointJump";

    case CartesianIkPathStatus::TimeLimit:
        return "TimeLimit";

    case CartesianIkPathStatus::NumericalFailure:
        return "NumericalFailure";
    }

    return "Unknown";
}

/**
 * 单点 numerical IK 的测试参数。
 *
 * 这里仍然只是数值测试参数，
 * 不表示真实机器人能够达到这些机械精度。
 */
IkOptions testIkOptions()
{
    IkOptions options;

    options.position_tolerance = 1e-6; // m
    options.rotation_tolerance = 1e-5; // rad

    options.max_iterations = 400;

    /*
     * 单个笛卡尔采样点最多使用 0.5 s。
     * 整条路径还有自己的 total_time_budget。
     */
    options.time_budget =
        std::chrono::duration<double>{0.5};

    return options;
}

/**
 * 根据当前被测路径动态选择采样间距。
 *
 * 目的不是给机器人定义生产参数，
 * 而是让本测试稳定产生大约 9 个 segment，
 * 从而确保 solveCartesianLineIk() 确实多次调用 IK，
 * 而不是只测试起点和终点。
 *
 * length / 8.5：
 *
 *     ceil(length / (length / 8.5))
 *       = ceil(8.5)
 *       = 9
 *
 * 姿态变化同理。
 */
CartesianIkPathOptions testPathOptions(
    const CartesianLinePath &path)
{
    CartesianIkPathOptions options;

    const double translation =
        path.translationLength();

    const double rotation =
        path.rotationAngle();

    require(
        translation > 0.0 || rotation > 0.0,
        "Non-zero Cartesian test path expected");

    if (translation > 0.0)
    {
        options.max_translation_step =
            translation / 8.5;
    }

    if (rotation > 0.0)
    {
        options.max_rotation_step =
            rotation / 8.5;
    }

    /*
     * 正常情况下，相邻 1/9 路径点的关节变化应该远低于 0.10 rad。
     *
     * 这是测试用连续性阈值，不是实际机器人速度限制。
     */
    options.max_rotational_joint_step = 0.10;
    options.max_prismatic_joint_step = 0.01;

    options.start_position_tolerance = 1e-8;
    options.start_rotation_tolerance = 1e-8;

    options.max_segments = 100;

    options.total_time_budget =
        std::chrono::duration<double>{5.0};

    return options;
}

/**
 * 按与生产代码相同的定义计算理论 segment 数。
 *
 * 这里不是为了重新实现 planner，
 * 而是让测试明确检查：
 *
 *   平移采样要求
 *   与
 *   旋转采样要求
 *
 * 最终确实取两者中更严格的一个。
 */
std::size_t expectedSegmentCount(
    const CartesianLinePath &path,
    const CartesianIkPathOptions &options)
{
    const auto translation_segments =
        static_cast<std::size_t>(
            std::ceil(
                path.translationLength() /
                options.max_translation_step));

    const auto rotation_segments =
        static_cast<std::size_t>(
            std::ceil(
                path.rotationAngle() /
                options.max_rotation_step));

    return std::max(
        translation_segments,
        rotation_segments);
}

/**
 * 测试 1：
 *
 * 笛卡尔路径起点 == 终点。
 *
 * 此时根本不需要调用后续 IK：
 *
 *     segment_count = 0
 *     waypoint 数量 = 1
 *     q 就是 start_q
 */
void testZeroLengthPath(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const JointVector start_q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };

    require(
        constraints.contains(start_q),
        "Zero-length start q violates constraints");

    const Pose start_pose =
        forwardKinematics(
            model,
            start_q);

    const CartesianLinePath path{
        start_pose,
        start_pose};

    const auto result =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            start_q);

    require(
        result.status ==
            CartesianIkPathStatus::Completed,
        std::string{
            "Zero-length path failed: "} +
            statusName(result.status) +
            ", " +
            result.message);

    require(
        result.completed(),
        "Zero-length path success flag mismatch");

    require(
        result.segment_count == 0,
        "Zero-length path unexpectedly has segments");

    require(
        result.waypoints.size() == 1,
        "Zero-length path should contain only start waypoint");

    require(
        result.waypoints.front().q == start_q,
        "Zero-length path changed start q");

    requireNear(
        result.completed_progress,
        1.0,
        0.0,
        "Zero-length path did not complete");

    std::cout
        << "[PASS] Zero-length Cartesian path\n";
}

/**
 * 测试 2：
 *
 * 真正的连续 MoveL-IK 集成测试。
 *
 * start_q：
 *     当前真实左臂关节状态。
 *
 * reference_goal_q：
 *     只通过 FK 生成终点 Pose。
 *
 * 注意：
 *     reference_goal_q 不是 IK 的“正确答案”。
 *
 *     CartesianLinePath 只使用：
 *
 *         FK(start_q)
 *         FK(reference_goal_q)
 *
 *     然后中间所有直线路径 Pose 都必须由
 *     solveCartesianLineIk() 自己逐点求解。
 *
 * 整个结构：
 *
 *     start_q
 *        ↓ FK
 *     start Pose
 *        ↓
 *        ↓ CartesianLinePath
 *        ↓
 *     Pose1 → IK → q1
 *     Pose2 → IK → q2
 *     Pose3 → IK → q3
 *     ...
 *     goal  → IK → qN
 */
void testRealLeftArmCartesianLine(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const JointVector start_q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };

    /*
     * 与 start_q 相邻的一组合法构型。
     *
     * 这里只用它制造一个真实可达的终点 Pose。
     */
    const JointVector reference_goal_q{
         0.40,
        -0.29,
         0.43,
        -0.75,
         0.41,
         0.28,
        -0.17
    };

    require(
        constraints.contains(start_q),
        "Cartesian start q violates constraints");

    require(
        constraints.contains(reference_goal_q),
        "Cartesian reference goal q violates constraints");

    const Pose start_pose =
        forwardKinematics(
            model,
            start_q);

    const Pose goal_pose =
        forwardKinematics(
            model,
            reference_goal_q);

    const CartesianLinePath path{
        start_pose,
        goal_pose};

    const CartesianIkPathOptions path_options =
        testPathOptions(path);

    const IkOptions ik_options =
        testIkOptions();

    const std::size_t expected_segments =
        expectedSegmentCount(
            path,
            path_options);

    /*
     * 这个测试必须真的包含多个中间点。
     *
     * 如果只有一两个 segment，
     * 就没有充分验证“上一点解作为下一点 seed”的机制。
     */
    require(
        expected_segments >= 5,
        "Cartesian integration test has too few segments");

    const auto result =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            start_q,
            path_options,
            ik_options);

    require(
        result.status ==
            CartesianIkPathStatus::Completed,
        std::string{
            "Real Cartesian line failed: "} +
            statusName(result.status) +
            ", message=" +
            result.message);

    require(
        result.completed(),
        "Completed Cartesian path has false completed()");

    require(
        result.segment_count ==
            expected_segments,
        "Unexpected Cartesian segment count");

    require(
        result.waypoints.size() ==
            result.segment_count + 1,
        "Waypoint count should be segment_count + 1");

    require(
        !result.failed_progress.has_value(),
        "Completed path unexpectedly records failed_progress");

    require(
        !result.failed_ik_status.has_value(),
        "Completed path unexpectedly records failed IK");

    requireNear(
        result.completed_progress,
        1.0,
        0.0,
        "Completed path did not reach progress 1");

    require(
        result.waypoints.front().q ==
            start_q,
        "First Cartesian waypoint changed start_q");

    std::size_t total_ik_iterations = 0;

    double measured_max_rotational_step = 0.0;
    double measured_max_prismatic_step = 0.0;

    /*
     * 对每一个 waypoint 做完整回验。
     */
    for (std::size_t i = 0;
         i < result.waypoints.size();
         ++i)
    {
        const auto &waypoint =
            result.waypoints[i];

        const double expected_progress =
            (i == result.segment_count)
                ? 1.0
                : static_cast<double>(i) /
                      static_cast<double>(
                          result.segment_count);

        requireNear(
            waypoint.progress,
            expected_progress,
            1e-15,
            "Unexpected waypoint progress");

        /*
         * 重新从 CartesianLinePath 取理论目标。
         *
         * 检查 result 里面保存的 target_pose
         * 没有在 planner 中被意外修改。
         */
        const Pose expected_target =
            path.sample(
                expected_progress);

        require(
            (waypoint.target_pose.position -
             expected_target.position)
                    .norm() <=
                1e-12,
            "Stored Cartesian target position mismatch");

        require(
            waypoint.target_pose.orientation
                    .angularDistance(
                        expected_target.orientation) <=
                1e-12,
            "Stored Cartesian target orientation mismatch");

        /*
         * 最关键的一步：
         *
         * q waypoint
         *      ↓ FK
         * actual_pose
         *
         * 然后与这一个 progress 的理论笛卡尔 Pose 比较。
         *
         * 也就是说：
         *
         *     不是只检查最终终点；
         *     中间每一个点都必须在规定的直线路径上。
         */
        require(
            constraints.contains(
                waypoint.q),
            "Cartesian waypoint violates joint bounds");

        const Pose actual_pose =
            forwardKinematics(
                model,
                waypoint.q);

        const double position_error =
            (expected_target.position -
             actual_pose.position)
                .stableNorm();

        const double rotation_error =
            actual_pose.orientation
                .angularDistance(
                    expected_target.orientation);

        require(
            std::isfinite(position_error) &&
                std::isfinite(rotation_error),
            "Non-finite Cartesian waypoint verification error");

        /*
         * 第 0 点是 start_q，本身由 FK 生成，所以应该非常接近 0。
         *
         * 后续点则按 numerical IK 的成功容差验收。
         */
        if (i == 0)
        {
            require(
                position_error <= 1e-12 &&
                    rotation_error <= 1e-12,
                "Cartesian start waypoint verification failed");
        }
        else
        {
            require(
                position_error <=
                    ik_options.position_tolerance,
                "Cartesian waypoint position error too large");

            require(
                rotation_error <=
                    ik_options.rotation_tolerance,
                "Cartesian waypoint rotation error too large");

            require(
                waypoint.position_error <=
                    ik_options.position_tolerance,
                "Stored IK position error exceeds tolerance");

            require(
                waypoint.rotation_error <=
                    ik_options.rotation_tolerance,
                "Stored IK rotation error exceeds tolerance");

            total_ik_iterations +=
                waypoint.ik_iterations;
        }

        /*
         * 再独立检查相邻关节 waypoint 的连续性。
         *
         * 注意：
         *     不是只相信 solveCartesianLineIk()
         *     自己报告的 max_observed_*。
         */
        if (i > 0)
        {
            const auto &previous =
                result.waypoints[i - 1];

            for (std::size_t q_index = 0;
                 q_index < kJointCount;
                 ++q_index)
            {
                const double delta =
                    std::abs(
                        waypoint.q[q_index] -
                        previous.q[q_index]);

                bool found_joint = false;
                UrdfJointType type =
                    UrdfJointType::Fixed;

                for (const auto &joint :
                     model.chain().joints)
                {
                    if (joint.q_index &&
                        *joint.q_index ==
                            q_index)
                    {
                        found_joint = true;
                        type = joint.type;
                        break;
                    }
                }

                require(
                    found_joint,
                    "Could not resolve joint type by q_index");

                if (type ==
                    UrdfJointType::Prismatic)
                {
                    measured_max_prismatic_step =
                        std::max(
                            measured_max_prismatic_step,
                            delta);

                    require(
                        delta <=
                            path_options
                                .max_prismatic_joint_step,
                        "Prismatic joint jump detected in successful path");
                }
                else
                {
                    measured_max_rotational_step =
                        std::max(
                            measured_max_rotational_step,
                            delta);

                    require(
                        delta <=
                            path_options
                                .max_rotational_joint_step,
                        "Rotational joint jump detected in successful path");
                }
            }
        }
    }

    /*
     * 非零路径必须真正运行过 numerical IK。
     *
     * 如果总 iteration == 0，
     * 说明测试没有真正覆盖逐点求解流程。
     */
    require(
        total_ik_iterations > 0,
        "Cartesian path unexpectedly required zero total IK iterations");

    requireNear(
        result.max_observed_rotational_joint_step,
        measured_max_rotational_step,
        1e-12,
        "Reported maximum rotational joint step mismatch");

    requireNear(
        result.max_observed_prismatic_joint_step,
        measured_max_prismatic_step,
        1e-12,
        "Reported maximum prismatic joint step mismatch");

    /*
     * 最终只检查末端 Pose，
     * 不要求最终 q == reference_goal_q。
     *
     * 因为 7DOF 可以有不同关节解得到同一个末端 Pose。
     */
    const Pose final_pose =
        forwardKinematics(
            model,
            result.waypoints.back().q);

    const double final_position_error =
        (goal_pose.position -
         final_pose.position)
            .stableNorm();

    const double final_rotation_error =
        final_pose.orientation
            .angularDistance(
                goal_pose.orientation);

    require(
        final_position_error <=
            ik_options.position_tolerance,
        "Final Cartesian position did not reach goal");

    require(
        final_rotation_error <=
            ik_options.rotation_tolerance,
        "Final Cartesian orientation did not reach goal");

    std::cout
        << std::fixed
        << std::setprecision(9)
        << "[PASS] Real left-arm Cartesian line\n"
        << "  translation length [m]: "
        << path.translationLength()
        << '\n'
        << "  rotation angle [rad]: "
        << path.rotationAngle()
        << '\n'
        << "  segments: "
        << result.segment_count
        << '\n'
        << "  waypoints: "
        << result.waypoints.size()
        << '\n'
        << "  total IK iterations: "
        << total_ik_iterations
        << '\n'
        << "  max joint step [rad]: "
        << measured_max_rotational_step
        << '\n'
        << "  final position error [m]: "
        << final_position_error
        << '\n'
        << "  final rotation error [rad]: "
        << final_rotation_error
        << '\n';
}

/**
 * 测试 3：
 *
 * path.start() 与 start_q 的 FK 不一致时必须拒绝。
 *
 * 如果不做这个检查，会出现：
 *
 *     实际机器人在 A
 *     规划器却认为路径从 B 开始
 *
 * 相当于路径起点发生瞬移。
 */
void testStartPoseMismatch(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const JointVector start_q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };

    const Pose actual_start =
        forwardKinematics(
            model,
            start_q);

    Pose wrong_start =
        actual_start;

    wrong_start.position.x() +=
        0.005; // 故意错 5 mm。

    const CartesianLinePath path{
        wrong_start,
        wrong_start};

    const auto result =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            start_q);

    require(
        result.status ==
            CartesianIkPathStatus::StartPoseMismatch,
        std::string{
            "Expected StartPoseMismatch, got "} +
            statusName(result.status) +
            ": " +
            result.message);

    require(
        result.waypoints.empty(),
        "Start mismatch should fail before accepting waypoints");

    std::cout
        << "[PASS] Cartesian start-pose mismatch rejected\n";
}

/**
 * 测试 4：
 *
 * 故意把允许的相邻关节跳变量设得极小，
 * 确认路径规划器不会因为每个 Pose 的 IK 都成功，
 * 就忽略关节空间的不连续。
 */
void testJointJumpGuard(
    const ValidatedUrdfChain<kJointCount> &model,
    const JointPositionConstraints<kJointCount> &constraints)
{
    const JointVector start_q{
         0.35,
        -0.25,
         0.40,
        -0.80,
         0.45,
         0.25,
        -0.20
    };

    const JointVector reference_goal_q{
         0.40,
        -0.29,
         0.43,
        -0.75,
         0.41,
         0.28,
        -0.17
    };

    const CartesianLinePath path{
        forwardKinematics(model, start_q),
        forwardKinematics(model, reference_goal_q)};

    auto path_options =
        testPathOptions(path);

    /*
     * 1e-8 rad 几乎禁止任何真实关节变化。
     *
     * 非零笛卡尔路径的第一批 IK 解必然需要某些关节发生变化，
     * 因而应该触发 JointJump。
     */
    path_options.max_rotational_joint_step =
        1e-8;

    const auto result =
        solveCartesianLineIk(
            model,
            constraints,
            path,
            start_q,
            path_options,
            testIkOptions());

    require(
        result.status ==
            CartesianIkPathStatus::JointJump,
        std::string{
            "Expected JointJump, got "} +
            statusName(result.status) +
            ": " +
            result.message);

    require(
        result.failed_progress.has_value(),
        "JointJump result has no failed_progress");

    require(
        *result.failed_progress > 0.0 &&
            *result.failed_progress <= 1.0,
        "Invalid JointJump failed_progress");

    require(
        result.completed_progress < 1.0,
        "JointJump path incorrectly reported complete");

    std::cout
        << "[PASS] Cartesian joint-jump guard\n";
}

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: motion_left_arm_cartesian_ik_test <robot.urdf>\n";

        return 2;
    }

    try
    {
        /*
         * 和上一轮真实 7DOF IK 测试使用同一条链：
         *
         *     WAIST_Y_S
         *        ↓
         *     L_WRIST_R_S
         *
         * 所以这里的笛卡尔 Pose 仍然是：
         *
         *     L_WRIST_R_S 相对 WAIST_Y_S
         *
         * 还不是额外工具 TCP 相对整机 base_link。
         */
        const UrdfChain raw_chain =
            loadUrdfChain(
                argv[1],
                "WAIST_Y_S",
                "L_WRIST_R_S",
                kJointCount);

        const ValidatedUrdfChain<kJointCount> model{
            raw_chain};

        const JointPositionConstraints<kJointCount> constraints{
            model};

        testZeroLengthPath(
            model,
            constraints);

        testRealLeftArmCartesianLine(
            model,
            constraints);

        testStartPoseMismatch(
            model,
            constraints);

        testJointJumpGuard(
            model,
            constraints);

        std::cout
            << "\nAll real left-arm Cartesian IK tests passed.\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Real left-arm Cartesian IK test failed: "
            << error.what()
            << '\n';

        return 1;
    }
    catch (...)
    {
        std::cerr
            << "Real left-arm Cartesian IK test failed: "
            << "unknown exception\n";

        return 1;
    }
}