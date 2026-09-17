#include "motion/kinematics/numerical_ik.hpp"
#include "motion/kinematics/urdf_chain.hpp"

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

    constexpr std::size_t kJointCount = 7;
    using JointVector = std::array<double, kJointCount>;

    /**
     * 当前真实左臂链：
     *
     *   WAIST_Y_S
     *      ↓ L_SHOULDER_P
     *   L_SHOULDER_P_S
     *      ↓ L_SHOULDER_R
     *   L_SHOULDER_R_S
     *      ↓ L_SHOULDER_Y
     *   L_SHOULDER_Y_S
     *      ↓ L_ELBOW_Y
     *   L_ELBOW_Y_S
     *      ↓ L_WRIST_P
     *   L_WRIST_P_S
     *      ↓ L_WRIST_Y
     *   L_WRIST_Y_S
     *      ↓ L_WRIST_R
     *   L_WRIST_R_S
     *
     * 数组顺序必须与 q[0]~q[6] 完全一致。
     */
    struct ExpectedJoint
    {
        const char *name;
        double lower;
        double upper;
    };

    constexpr std::array<ExpectedJoint, kJointCount> kExpectedJoints{{
        {"L_SHOULDER_P", -3.14, 3.14},
        {"L_SHOULDER_R", -1.57, 1.57},
        {"L_SHOULDER_Y", -3.14, 3.14},
        {"L_ELBOW_Y", -2.43, 0.79},
        {"L_WRIST_P", -3.14, 3.14},
        {"L_WRIST_Y", -0.94, 0.94},
        {"L_WRIST_R", -2.00, 1.48},
    }};

    /** 测试辅助函数：条件不满足就让整个测试失败。 */
    void require(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }

    /** 测试浮点数是否足够接近。 */
    void requireNear(
        const double actual,
        const double expected,
        const double tolerance,
        const std::string &message)
    {
        if (!std::isfinite(actual) ||
            std::abs(actual - expected) > tolerance)
        {
            throw std::runtime_error(
                message +
                ": actual=" + std::to_string(actual) +
                ", expected=" + std::to_string(expected));
        }
    }

    /**
     * 真实左臂 IK 集成测试使用的参数。
     *
     * 这些容差只用于本次数值回归测试：
     *   position_tolerance = 1e-5 m
     *   rotation_tolerance = 1e-4 rad
     *
     * 不能据此宣称真实机械臂具有 10 um 定位精度。
     *
     * time_budget 给到 3 s，是为了避免开发机上的测试因为调度抖动失败，
     * 也不是未来在线规划的性能指标。
     */
    IkOptions integrationOptions()
    {
        IkOptions options;

        options.position_tolerance = 1e-5;
        options.rotation_tolerance = 1e-4;

        options.max_iterations = 400;
        options.time_budget = std::chrono::duration<double>{3.0};

        return options;
    }

    /**
     * 检查 loadUrdfChain() 从真实 URDF 得到的左臂结构。
     *
     * 上游：
     *   main() 调用 loadUrdfChain() 后，把原始 UrdfChain 传进来。
     *
     * 检查：
     *   1. base/tip 是否正确；
     *   2. 是否正好得到 7 个关节；
     *   3. q 顺序是否正确；
     *   4. 是否都是 Revolute；
     *   5. q_index 是否与数组顺序一致；
     *   6. axis 是否已经归一化；
     *   7. URDF lower/upper 是否真正进入 position_bounds。
     *
     * 下游：
     *   全部通过后，才允许构造 ValidatedUrdfChain<7> 和约束对象。
     */
    void verifyLoadedLeftArm(const UrdfChain &chain)
    {
        require(
            chain.base_link == "WAIST_Y_S",
            "Unexpected left-arm base link");

        require(
            chain.tip_link == "L_WRIST_R_S",
            "Unexpected left-arm tip link");

        require(
            chain.joints.size() == kJointCount,
            "Expected exactly seven joints in the selected left-arm chain");

        require(
            chain.joint_names.size() == kJointCount,
            "Expected exactly seven active left-arm joints");

        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            const auto &expected = kExpectedJoints[i];
            const auto &joint = chain.joints[i];

            require(
                joint.name == expected.name,
                "Unexpected joint at q[" + std::to_string(i) + "]");

            require(
                chain.joint_names[i] == expected.name,
                "joint_names order mismatch at q[" + std::to_string(i) + "]");

            require(
                joint.type == UrdfJointType::Revolute,
                "Expected revolute joint: " + joint.name);

            require(
                joint.q_index.has_value(),
                "Active joint has no q_index: " + joint.name);

            require(
                *joint.q_index == i,
                "Unexpected q_index for joint: " + joint.name);

            require(
                joint.axis.allFinite(),
                "Non-finite joint axis: " + joint.name);

            requireNear(
                joint.axis.norm(),
                1.0,
                1e-12,
                "Joint axis is not normalized: " + joint.name);

            require(
                joint.position_bounds.has_value(),
                "URDF position bounds were not loaded: " + joint.name);

            requireNear(
                joint.position_bounds->lower(),
                expected.lower,
                1e-12,
                "Unexpected lower bound: " + joint.name);

            requireNear(
                joint.position_bounds->upper(),
                expected.upper,
                1e-12,
                "Unexpected upper bound: " + joint.name);
        }

        std::cout
            << "[PASS] Real left-arm URDF structure and limits\n";
    }

    /**
     * 对一个 IK 成功结果重新做独立验收。
     *
     * 不只是相信 result.status：
     *
     *   result.q
     *      ↓
     *   再次 FK
     *      ↓
     *   与 target 比较
     *
     * 同时确认结果仍处于有效关节范围内。
     *
     * 注意：
     *   七自由度是冗余机构。
     *   因此不要求求解结果 q_result 必须等于生成 target 时的 q_goal。
     *
     *   只要：
     *       FK(q_result) == target（容差内）
     *       且 q_result 满足限位
     *
     *   就是一组有效逆解。
     */
    void verifyIkResult(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints,
        const Pose &target,
        const IkOptions &options,
        const IkResult<kJointCount> &result,
        const std::string &case_name)
    {
        require(
            result.status == IkStatus::Converged,
            case_name +
                ": IK did not converge, status=" +
                std::to_string(static_cast<int>(result.status)) +
                ", message=" + result.message);

        require(
            result.converged(),
            case_name + ": result status/success mismatch");

        require(
            result.candidate_q.has_value(),
            case_name + ": converged result has no joint solution");

        const JointVector &solved_q = *result.candidate_q;

        require(
            constraints.contains(solved_q),
            case_name + ": returned joint solution violates constraints");

        const Pose solved_pose =
            forwardKinematics(model, solved_q);

        const double position_error =
            (target.position - solved_pose.position).norm();

        const double rotation_error =
            solved_pose.orientation.angularDistance(
                target.orientation);

        require(
            std::isfinite(position_error) &&
                std::isfinite(rotation_error),
            case_name + ": verification produced non-finite error");

        require(
            position_error <= options.position_tolerance,
            case_name + ": FK position verification failed");

        require(
            rotation_error <= options.rotation_tolerance,
            case_name + ": FK rotation verification failed");

        std::cout
            << std::fixed << std::setprecision(8)
            << "[PASS] " << case_name << '\n'
            << "  iterations: " << result.iterations << '\n'
            << "  pose evaluations: " << result.pose_evaluations << '\n'
            << "  position error [m]: " << position_error << '\n'
            << "  rotation error [rad]: " << rotation_error << '\n'
            << "  solved q:";

        for (const double value : solved_q)
        {
            std::cout << ' ' << value;
        }

        std::cout << '\n';
    }

    /**
     * 运行一个真实七轴 IK 用例。
     *
     * q_goal：
     *   只用于通过 FK 生成一个“确定属于当前模型可达空间”的 target。
     *   q_goal 不传给 IK，因此求解器不知道参考答案。
     *
     * seed：
     *   IK 真正看到的初始关节位置。
     *
     * 流程：
     *
     *   q_goal
     *      ↓ FK
     *   target
     *
     *   seed + target
     *      ↓ IK
     *   solved_q
     *      ↓ FK
     *   检查是否重新得到 target
     *
     * 这样能把“目标是否本来就不可达”这个因素从测试中排除。
     */
    void runReachableCase(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints,
        const JointVector &q_goal,
        const JointVector &seed,
        const IkOptions &options,
        const std::string &case_name)
    {
        require(
            constraints.contains(q_goal),
            case_name + ": test q_goal violates joint bounds");

        require(
            constraints.contains(seed),
            case_name + ": test seed violates joint bounds");

        const Pose target =
            forwardKinematics(model, q_goal);

        /*
         * 确认 seed 并不是已经处在目标容差内。
         * 否则这个测试只会走“0 次迭代成功”，没有验证真正的 IK 迭代。
         */
        const Pose seed_pose =
            forwardKinematics(model, seed);

        const double initial_position_error =
            (target.position - seed_pose.position).norm();

        const double initial_rotation_error =
            seed_pose.orientation.angularDistance(
                target.orientation);

        require(
            initial_position_error > options.position_tolerance ||
                initial_rotation_error > options.rotation_tolerance,
            case_name + ": seed is already inside target tolerance");

        const auto result =
            solveNumericalIk(
                model,
                constraints,
                target,
                seed,
                options);

        verifyIkResult(
            model,
            constraints,
            target,
            options,
            result,
            case_name);
    }

    /**
     * 检查目标本来就是 seed 对应位姿时：
     *
     *   solver 不应该无缘无故寻找其他七轴解，
     *   应在初始误差检查时直接返回。
     */
    void testAlreadyAtTarget(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints,
        const IkOptions &options)
    {
        const JointVector q{
            0.35,
            -0.25,
            0.40,
            -0.80,
            0.45,
            0.25,
            -0.20};

        require(
            constraints.contains(q),
            "Already-at-target q violates constraints");

        const Pose target =
            forwardKinematics(model, q);

        const auto result =
            solveNumericalIk(
                model,
                constraints,
                target,
                q,
                options);

        verifyIkResult(
            model,
            constraints,
            target,
            options,
            result,
            "Real left arm: seed already at target");

        require(
            result.iterations == 0,
            "Already-at-target case should require zero IK iterations");

        require(
            *result.candidate_q == q,
            "Already-at-target case unexpectedly changed q");
    }

    /**
     * 多组真实七轴构型回归。
     *
     * 所有 q_goal 都明确位于真实 URDF lower/upper 内；
     * seed 与对应 q_goal 较接近，但不是同一个构型。
     *
     * 目标都是通过真实左臂 FK 生成，因此确定可达。
     *
     * 不要求最终 q 与 q_goal 相同：
     * 七自由度可能存在其他满足同一个末端位姿的合法解。
     */
    void testNearbySeeds(
        const ValidatedUrdfChain<kJointCount> &model,
        const JointPositionConstraints<kJointCount> &constraints,
        const IkOptions &options)
    {
        struct Case
        {
            const char *name;
            JointVector q_goal;
            JointVector seed;
        };

        const std::array<Case, 3> cases{{{"Real left arm case A",
                                          {0.35,
                                           -0.25,
                                           0.40,
                                           -0.80,
                                           0.45,
                                           0.25,
                                           -0.20},
                                          {0.40,
                                           -0.29,
                                           0.43,
                                           -0.75,
                                           0.41,
                                           0.28,
                                           -0.17}},
                                         {"Real left arm case B",
                                          {-0.50,
                                           0.35,
                                           -0.45,
                                           -1.10,
                                           -0.35,
                                           -0.30,
                                           0.40},
                                          {-0.44,
                                           0.31,
                                           -0.49,
                                           -1.04,
                                           -0.31,
                                           -0.26,
                                           0.36}},
                                         {/*
                                           * 这组让肘和腕的一部分位置更靠近 URDF 上界，
                                           * 用来覆盖约束开始产生实际影响的区域。
                                           *
                                           * 仍然没有直接贴边，避免测试本身人为制造退化情况。
                                           */
                                          "Real left arm case C near limits",
                                          {0.90,
                                           -0.70,
                                           0.80,
                                           0.55,
                                           -0.60,
                                           0.75,
                                           1.10},
                                          {0.84,
                                           -0.64,
                                           0.74,
                                           0.49,
                                           -0.54,
                                           0.69,
                                           1.04}}}};

        for (const auto &test_case : cases)
        {
            runReachableCase(
                model,
                constraints,
                test_case.q_goal,
                test_case.seed,
                options,
                test_case.name);
        }
    }

} // namespace

int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cerr
            << "Usage: motion_left_arm_ik_test <robot.urdf>\n";

        return 2;
    }

    try
    {
        /*
         * 1. 从真实 T170C URDF 提取左臂链。
         *
         * 结果坐标约定：
         *   base = WAIST_Y_S
         *   tip  = L_WRIST_R_S
         *
         * 不自动变成整机 base_link，也不自动附加掌心 TCP。
         */
        const UrdfChain raw_chain =
            loadUrdfChain(
                argv[1],
                "WAIST_Y_S",
                "L_WRIST_R_S",
                kJointCount);

        /*
         * 2. 先检查真实 URDF 的结构、q 顺序和 lower/upper。
         */
        verifyLoadedLeftArm(raw_chain);

        /*
         * 3. 建立只读几何模型。
         */
        const ValidatedUrdfChain<kJointCount> model{
            raw_chain};

        /*
         * 4. 本测试不额外提供配置范围。
         *
         * 因此有效位置约束完全来自真实 URDF。
         * 如果某个 revolute 关节没有成功加载 lower/upper，
         * 这里构造就应该失败，而不是把它当作无限制。
         */
        const JointPositionConstraints<kJointCount> constraints{
            model};

        const IkOptions options =
            integrationOptions();

        /*
         * 5. 初值已经是解时，检查零迭代返回。
         */
        testAlreadyAtTarget(
            model,
            constraints,
            options);

        /*
         * 6. 多组真实七轴目标，从附近 seed 做真正的 IK 迭代。
         */
        testNearbySeeds(
            model,
            constraints,
            options);

        std::cout
            << "\nAll real left-arm IK integration tests passed.\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Real left-arm IK integration test failed: "
            << error.what()
            << '\n';

        return 1;
    }
    catch (...)
    {
        std::cerr
            << "Real left-arm IK integration test failed: "
            << "unknown exception\n";

        return 1;
    }
}