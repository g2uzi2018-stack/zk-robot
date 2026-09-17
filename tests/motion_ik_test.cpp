#include "motion/kinematics/numerical_ik.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
    using namespace robot::motion;
    const double kPi = std::acos(-1.0);

    /** 各测试调用：条件不成立就抛异常给 main；不使用会被 NDEBUG 关闭的 assert。 */
    void require(const bool condition, const std::string &message)
    {
        if (!condition)
            throw std::runtime_error(message);
    }

    /** 各测试调用：返回数值回归参数；宽松时间预算不是实时性能指标。 */
    IkOptions testOptions()
    {
        IkOptions options;
        options.position_tolerance = 1e-6;
        options.rotation_tolerance = 1e-6;
        options.time_budget = std::chrono::duration<double>{5.0};
        return options;
    }

    /** 各测试调用：输入真实求解结果和预期状态；只检查停止原因，不代表位姿验收。 */
    template <std::size_t N>
    void expectStatus(
        const IkResult<N> &result, const IkStatus expected, const std::string &label)
    {
        require(result.status == expected,
                label + ": unexpected status=" + std::to_string(static_cast<int>(result.status)) +
                    ", message=" + result.message);
        require(result.converged() == (expected == IkStatus::Converged),
                label + ": inconsistent success flag");
    }

    /**
     * 成功用例调用：重新做 FK 验收，不只相信 result 中的误差报告。
     * 输入来自同一次求解的 model、constraints、target、options 和 result。
     * 位置用向量差，朝向用角距离；不调用 poseErrorInBase 来重复同一误差实现。
     * 同时检查候选有限、四元数单位长度、位置范围，以及诊断误差是否一致。
     * 通过时正常返回，失败抛异常；不能证明 FK 模型本身与实物一致。
     */
    template <std::size_t N>
    void verifySolution(
        const ValidatedUrdfChain<N> &model,
        const JointPositionConstraints<N> &constraints,
        const Pose &target, const IkOptions &options, const IkResult<N> &result)
    {
        expectStatus(result, IkStatus::Converged, "Solution");
        require(result.candidate_q.has_value(), "Successful result has no q");
        require(constraints.contains(*result.candidate_q), "Returned q violates bounds");
        const Pose actual = forwardKinematics(model, *result.candidate_q);
        require(actual.position.allFinite() && actual.orientation.coeffs().allFinite(),
                "Returned pose is non-finite");
        require(std::abs(actual.orientation.norm() - 1.0) <= 1e-12,
                "FK quaternion is not unit length");
        const double position_error = (target.position - actual.position).norm();
        const double rotation_error = actual.orientation.angularDistance(target.orientation);
        require(std::isfinite(position_error) && std::isfinite(rotation_error),
                "Recomputed error is non-finite");
        require(position_error <= options.position_tolerance &&
                    rotation_error <= options.rotation_tolerance,
                "Target tolerance not satisfied");
        require(std::isfinite(result.position_error) && std::isfinite(result.rotation_error) &&
                    std::abs(result.position_error - position_error) <= 1e-10 &&
                    std::abs(result.rotation_error - rotation_error) <= 1e-10,
                "Reported error disagrees with FK verification");
    }

    /**
     * 平面测试调用：构造三活动关节、两段 0.25 m 连杆及 0.10 m 固定工具。
     * 三个局部轴均为 +Z，位置范围均为 [-pi, pi]；q 顺序为 joint_0/1/2。
     * 全零时末端 (0.60,0,0)，朝向为单位旋转。不是实际左臂参数。
     */
    UrdfChain makePlanarChain()
    {
        UrdfChain chain{"base", "tool", {}, {}};
        std::string parent = chain.base_link;
        for (std::size_t i = 0; i < 3; ++i)
        {
            UrdfChainJoint joint;
            joint.name = "joint_" + std::to_string(i);
            joint.parent_link = parent;
            joint.child_link = "link_" + std::to_string(i);
            joint.type = UrdfJointType::Revolute;
            joint.axis = Eigen::Vector3d::UnitZ();
            joint.origin.position.x() = (i == 0) ? 0.0 : 0.25;
            joint.q_index = i;
            joint.position_bounds.emplace(-kPi, kPi);
            chain.joint_names.push_back(joint.name);
            chain.joints.push_back(joint);
            parent = joint.child_link;
        }
        UrdfChainJoint tool;
        tool.name = "tool_mount";
        tool.parent_link = parent;
        tool.child_link = chain.tip_link;
        tool.type = UrdfJointType::Fixed;
        tool.origin.position.x() = 0.10;
        chain.joints.push_back(tool);
        return chain;
    }

    /**
     * 移动关节测试调用：局部 X 经安装朝向转成基准 +Y。
     * q 是移动距离，范围 [-0.20,0.20] m，安装位置 (0.30,0.10,0) m。
     */
    UrdfChain makeSliderChain()
    {
        UrdfChainJoint slider;
        slider.name = "slider";
        slider.parent_link = "base";
        slider.child_link = "carriage";
        slider.type = UrdfJointType::Prismatic;
        slider.axis = Eigen::Vector3d::UnitX();
        slider.origin.position = Eigen::Vector3d{0.30, 0.10, 0.0};
        slider.origin.orientation = Eigen::Quaterniond{
            Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
        slider.q_index = 0;
        slider.position_bounds.emplace(-0.20, 0.20);
        return UrdfChain{"base", "carriage", {slider}, {slider.name}};
    }

    /** main 调用：初值已经到达目标时，不应无故寻找另一组构型。 */
    void testAlreadyAtTarget()
    {
        const ValidatedUrdfChain<3> model{makePlanarChain()};
        const JointPositionConstraints<3> constraints{model};
        const std::array<double, 3> seed{0.2, -0.7, 0.5};
        const auto seed_before = seed;
        const Pose target = forwardKinematics(model, seed);
        const auto options = testOptions();
        const auto result = solveNumericalIk(model, constraints, target, seed, options);
        verifySolution(model, constraints, target, options, result);
        require(result.iterations == 0 && *result.candidate_q == seed,
                "Already-solved seed was unnecessarily changed");
        require(seed == seed_before, "Solver modified seed");
        std::cout << "[PASS] Already at target: zero iterations\n";
    }

    /**
     * main 调用：同一目标，差初值停住，两组较好初值分别收敛到两个构型。
     * 手算目标：q=(theta,-2*theta,theta)，theta=0.6 rad；
     * 末端 x=0.50*cos(theta)+0.10，y=0，朝向为单位旋转。
     * 反号构型也达到同一目标。这是独立的几何预期，不要求求解器返回指定答案。
     * 三次调用是测试显式发起的；不是求解器内部已经实现了多初值重启。
     */
    void testSeedDependence()
    {
        const ValidatedUrdfChain<3> model{makePlanarChain()};
        const JointPositionConstraints<3> constraints{model};
        const auto options = testOptions();
        Pose target{};
        target.position = Eigen::Vector3d{0.50 * std::cos(0.6) + 0.10, 0.0, 0.0};

        // 从完全伸直位置沿轴线向内收：一阶雅可比给不出径向缩短方向。
        const std::array<double, 3> straight{0.0, 0.0, 0.0};
        const auto stuck = solveNumericalIk(model, constraints, target, straight, options);
        expectStatus(stuck, IkStatus::NoProgress, "Straight seed");
        require(stuck.candidate_q.has_value() && *stuck.candidate_q == straight,
                "Unexpected candidate at straight seed");
        require(stuck.position_error > options.position_tolerance,
                "Unreached target was reported as reached");
        std::cout << "[PASS] Reachable target, straight seed: NoProgress\n";

        const std::array<double, 3> seed_a{0.25, -0.50, 0.25};
        const std::array<double, 3> seed_b{-0.25, 0.50, -0.25};
        const auto a = solveNumericalIk(model, constraints, target, seed_a, options);
        const auto b = solveNumericalIk(model, constraints, target, seed_b, options);
        verifySolution(model, constraints, target, options, a);
        verifySolution(model, constraints, target, options, b);
        require((*a.candidate_q)[1] < -0.1 && (*b.candidate_q)[1] > 0.1,
                "Expected two distinct elbow branches");
        std::cout << std::fixed << std::setprecision(6)
                  << "[PASS] Same target, two seeds: both Converged\n"
                  << "  q_a: " << (*a.candidate_q)[0] << ' ' << (*a.candidate_q)[1]
                  << ' ' << (*a.candidate_q)[2] << '\n'
                  << "  q_b: " << (*b.candidate_q)[0] << ' ' << (*b.candidate_q)[1]
                  << ' ' << (*b.candidate_q)[2] << '\n';
    }

    /** main 调用：移动关节可达目标应收敛；越界目标不能以边界候选冒充成功。 */
    void testPositionLimits()
    {
        const ValidatedUrdfChain<1> model{makeSliderChain()};
        const JointPositionConstraints<1> constraints{model};
        const auto options = testOptions();
        const std::array<double, 1> seed{0.0};
        Pose target = model.chain().joints.front().origin;
        target.position.y() += 0.08;
        const auto reached = solveNumericalIk(model, constraints, target, seed, options);
        verifySolution(model, constraints, target, options, reached);
        require(std::abs((*reached.candidate_q)[0] - 0.08) <= options.position_tolerance,
                "Prismatic position mismatch");
        std::cout << "[PASS] Prismatic target: Converged\n";

        target.position.y() = 0.40; // 需要 q=0.30 m，但上界只有 0.20 m。
        const auto blocked = solveNumericalIk(model, constraints, target, seed, options);
        expectStatus(blocked, IkStatus::NoProgress, "Target beyond position limit");
        require(blocked.candidate_q.has_value() && constraints.contains(*blocked.candidate_q),
                "Failed result lost its feasible diagnostic candidate");
        require(std::abs((*blocked.candidate_q)[0] - 0.20) < 1e-9 &&
                    std::abs(blocked.position_error - 0.10) < 1e-9,
                "Incorrect limit-boundary diagnostics");
        std::cout << "[PASS] Beyond-limit target: NoProgress, not success\n";
    }

    /** main 调用：非法输入、越界初值必须拒绝；不能静默归零或裁剪成另一个初值。 */
    void testInvalidInputs()
    {
        const ValidatedUrdfChain<1> model{makeSliderChain()};
        const JointPositionConstraints<1> constraints{model};
        const auto options = testOptions();
        const std::array<double, 1> seed{0.0};
        const Pose target = model.chain().joints.front().origin;
        auto result = solveNumericalIk(model, constraints, target,
                                       std::array<double, 1>{0.21}, options);
        expectStatus(result, IkStatus::SeedOutOfBounds, "Out-of-bounds seed");
        require(!result.candidate_q, "Invalid seed produced a candidate");

        const std::array<double, 3> invalid_values{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
        for (double value : invalid_values)
        {
            result = solveNumericalIk(model, constraints, target,
                                      std::array<double, 1>{value}, options);
            expectStatus(result, IkStatus::InvalidInput, "Non-finite seed");
            Pose bad_target = target;
            bad_target.position.x() = value;
            result = solveNumericalIk(model, constraints, bad_target, seed, options);
            expectStatus(result, IkStatus::InvalidInput, "Non-finite target");
        }
        Pose bad_target = target;
        bad_target.orientation.coeffs().setZero();
        expectStatus(solveNumericalIk(model, constraints, bad_target, seed, options),
                     IkStatus::InvalidInput, "Zero quaternion");
        auto bad_options = options;
        bad_options.position_scale = 0.0;
        expectStatus(solveNumericalIk(model, constraints, target, seed, bad_options),
                     IkStatus::InvalidInput, "Zero position scale");
        std::cout << "[PASS] Invalid input and out-of-bounds seed rejection\n";
    }

    /** main 调用：预算耗尽与数值失败必须返回失败；本测试不证明硬实时截止。 */
    void testBudgetsAndNumerics()
    {
        const ValidatedUrdfChain<1> model{makeSliderChain()};
        const JointPositionConstraints<1> constraints{model};
        const std::array<double, 1> seed{0.0};
        Pose target = model.chain().joints.front().origin;
        target.position.y() += 0.18;
        auto options = testOptions();
        options.max_iterations = 1; // 每轮最多移动 0.02 m，故一轮不可能到达。
        const auto limited = solveNumericalIk(model, constraints, target, seed, options);
        expectStatus(limited, IkStatus::IterationLimit, "Iteration budget");
        require(limited.iterations == 1 && limited.candidate_q.has_value() &&
                    constraints.contains(*limited.candidate_q),
                "Invalid iteration-limit result");

        options = testOptions();
        options.time_budget = std::chrono::duration<double>{1e-12};
        expectStatus(solveNumericalIk(model, constraints, target, seed, options),
                     IkStatus::TimeLimit, "Time budget");

        options = testOptions();
        options.position_scale = 1e-300; // 参数有限且为正，但缩放后的平方代价溢出。
        expectStatus(solveNumericalIk(model, constraints, target, seed, options),
                     IkStatus::NumericalFailure, "Cost overflow");
        std::cout << "[PASS] Iteration/time budgets and numerical failure\n";
    }
} // namespace

int main()
{
    try
    {
        testAlreadyAtTarget();
        testSeedDependence();
        testPositionLimits();
        testInvalidInputs();
        testBudgetsAndNumerics();
        std::cout << "All numerical IK regression tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Numerical IK test failed: " << error.what() << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "Numerical IK test failed: unknown exception\n";
        return 1;
    }
}