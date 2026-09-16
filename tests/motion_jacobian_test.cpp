#include "motion/kinematics/geometric_jacobian.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>

namespace
{
    using namespace robot::motion;
    const double kPi = std::acos(-1.0);

    /** 差分测试调用：检查 FK 输出，不替 FK 修正非单位四元数。 */
    void expectValidPose(const Pose &pose)
    {
        const double norm = pose.orientation.norm();
        if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite() ||
            !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-12)
        {
            throw std::runtime_error("Finite difference received an invalid FK pose");
        }
    }

    /**
     * 仅用于测试的数值参考，不是生产雅可比实现。
     * 上游：测试函数提供模型、构型 q 和小扰动 h。
     * 对每个 q[i] 分别加/减 h，调用两次 FK；不移动真实机械臂。
     * h 对旋转关节的单位为 rad，对移动关节为 m，不是时间步长。
     *
     * 上三行：(p_plus - p_minus) / (2*h)。
     * 下三行：rotvec(R_plus * R_minus^T) / (2*h)。
     * rotvec = 单位旋转轴 * 旋转角，表达在 base 坐标系。
     * 不能用 RPY 或四元数系数直接相减代替角运动部分。
     *
     * 返回 6×N 数值参考给测试，与 geometricJacobian() 逐列比较。
     */
    template <std::size_t N>
    Eigen::Matrix<double, 6, N> finiteDifferenceJacobian(
        const ValidatedUrdfChain<N> &model,
        const std::array<double, N> &q,
        const double h)
    {
        if (!std::isfinite(h) || h <= 0.0 || h > 1e-3)
        {
            throw std::invalid_argument("Test step must satisfy 0 < h <= 1e-3");
        }

        Eigen::Matrix<double, 6, N> result = Eigen::Matrix<double, 6, N>::Zero();
        for (std::size_t i = 0; i < N; ++i)
        {
            auto plus = q;
            auto minus = q;
            plus[i] += h;
            minus[i] -= h;
            if (plus[i] == q[i] || minus[i] == q[i])
            {
                throw std::runtime_error("Test step is too small for this q value");
            }

            const Pose pose_plus = forwardKinematics(model, plus);
            const Pose pose_minus = forwardKinematics(model, minus);
            expectValidPose(pose_plus);
            expectValidPose(pose_minus);

            // 单位四元数的共轭等于逆；次序决定角度差在哪个坐标系表达。
            Eigen::Quaterniond delta =
                pose_plus.orientation * pose_minus.orientation.conjugate();
            delta.normalize(); // 只修正新做的旋转乘积的舍入误差。
            if (delta.w() < 0.0)
            {
                delta.coeffs() *= -1.0; // Q 和 -Q 表示同一个旋转，选短旋转。
            }
            const Eigen::AngleAxisd rotation_difference{delta};
            const Eigen::Index column = static_cast<Eigen::Index>(i);
            result.template block<3, 1>(0, column) =
                (pose_plus.position - pose_minus.position) / (2.0 * h);
            result.template block<3, 1>(3, column) =
                rotation_difference.axis() * (rotation_difference.angle() / (2.0 * h));
        }
        return result;
    }

    /**
     * 各测试调用：比较实际 J 和参考 J，返回最大线/角列误差。
     * 分别比较上下三行，不把不同量纲混成一个 6 维误差范数。
     * 容差是导数的数值比较阈值，不是真实机械臂的定位精度。
     * 返回数组：[0] 最大线运动部分误差，[1] 最大角运动部分误差。
     */
    template <int N>
    std::array<double, 2> expectJacobianNear(
        const Eigen::Matrix<double, 6, N> &actual,
        const Eigen::Matrix<double, 6, N> &expected,
        const double linear_tolerance,
        const double angular_tolerance,
        const std::string &message)
    {
        if (!actual.allFinite() || !expected.allFinite())
        {
            throw std::runtime_error(message + ": non-finite Jacobian");
        }
        std::array<double, 2> worst{0.0, 0.0};
        for (Eigen::Index i = 0; i < N; ++i)
        {
            const double linear_error =
                (actual.template block<3, 1>(0, i) -
                 expected.template block<3, 1>(0, i))
                    .norm();
            const double angular_error =
                (actual.template block<3, 1>(3, i) -
                 expected.template block<3, 1>(3, i))
                    .norm();
            if (!std::isfinite(linear_error) || !std::isfinite(angular_error) ||
                linear_error > linear_tolerance || angular_error > angular_tolerance)
            {
                std::cerr << message << ", column " << i
                          << ", linear error " << linear_error
                          << ", angular error " << angular_error << '\n';
                throw std::runtime_error("Jacobian comparison failed");
            }
            worst[0] = std::max(worst[0], linear_error);
            worst[1] = std::max(worst[1], angular_error);
        }
        return worst;
    }

    /** 失败路径测试调用：只有抛出指定类型或其派生异常，才算通过。 */
    template <typename ExpectedException, typename Action>
    void expectThrows(Action action, const std::string &message)
    {
        try
        {
            action();
        }
        catch (const ExpectedException &)
        {
            return;
        }
        catch (const std::exception &error)
        {
            throw std::runtime_error(message + ": wrong exception: " + error.what());
        }
        throw std::runtime_error(message + ": expected exception was not thrown");
    }

    /** 测试数据：两根局部 Z 转轴，0.30 m + 0.20 m，含固定末端。 */
    UrdfChain makeTwoLinkChain()
    {
        UrdfChainJoint shoulder;
        shoulder.name = "shoulder";
        shoulder.parent_link = "base";
        shoulder.child_link = "upper_arm";
        shoulder.type = UrdfJointType::Revolute;
        shoulder.axis = Eigen::Vector3d::UnitZ();
        shoulder.q_index = 0;

        UrdfChainJoint elbow;
        elbow.name = "elbow";
        elbow.parent_link = "upper_arm";
        elbow.child_link = "forearm";
        elbow.type = UrdfJointType::Revolute;
        elbow.origin.position = Eigen::Vector3d{0.30, 0.0, 0.0};
        elbow.axis = Eigen::Vector3d::UnitZ();
        elbow.q_index = 1;

        UrdfChainJoint tool;
        tool.name = "tool_mount";
        tool.parent_link = "forearm";
        tool.child_link = "tool";
        tool.type = UrdfJointType::Fixed;
        tool.origin.position = Eigen::Vector3d{0.20, 0.0, 0.0};
        return UrdfChain{
            "base", "tool", {shoulder, elbow, tool}, {"shoulder", "elbow"}};
    }

    /** main 调用：独立手算矩阵检查，不能只让 J 与 FK 相互证明正确。 */
    void testAnalyticCases()
    {
        const ValidatedUrdfChain<2> model{makeTwoLinkChain()};
        const std::array<double, 2> q{kPi / 6.0, kPi / 3.0};
        Eigen::Matrix<double, 6, 2> expected;
        expected << -0.35, -0.20,
            0.15 * std::sqrt(3.0), 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            1.0, 1.0;
        expectJacobianNear(geometricJacobian(model, q), expected,
                           1e-12, 1e-12, "Two-link analytic");
        expectJacobianNear(finiteDifferenceJacobian(model, q, 1e-6), expected,
                           2e-7, 2e-7, "Two-link difference");
        std::cout << "[PASS] Two-link analytic and finite difference\n";

        // 把两转轴放到同一位置，末端也在轴上：两列相同，完整 J 秩不足。
        UrdfChain coincident = makeTwoLinkChain();
        coincident.joints[1].origin.position.setZero();
        coincident.joints[2].origin.position.setZero();
        const ValidatedUrdfChain<2> singular_model{coincident};
        expected << 0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            0.0, 0.0,
            1.0, 1.0;
        expectJacobianNear(geometricJacobian(singular_model, q), expected,
                           1e-12, 1e-12, "Rank-deficient model");
        std::cout << "[PASS] Rank-deficient Jacobian is still computable\n";

        UrdfChainJoint slider;
        slider.name = "slider";
        slider.parent_link = "base";
        slider.child_link = "carriage";
        slider.type = UrdfJointType::Prismatic;
        slider.origin.position = Eigen::Vector3d{0.30, 0.10, 0.0};
        slider.origin.orientation = Eigen::Quaterniond{
            Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
        slider.axis = Eigen::Vector3d{2.0, 0.0, 0.0}; // 由模型初始化归一化。
        slider.q_index = 0;
        const UrdfChain source{"base", "carriage", {slider}, {slider.name}};
        const ValidatedUrdfChain<1> slider_model{source};
        const std::array<double, 1> slider_q{-0.20};
        Eigen::Matrix<double, 6, 1> slider_expected;
        slider_expected << 0.0, 1.0, 0.0, 0.0, 0.0, 0.0;
        expectJacobianNear(geometricJacobian(slider_model, slider_q), slider_expected,
                           1e-12, 1e-12, "Prismatic analytic");
        expectJacobianNear(finiteDifferenceJacobian(slider_model, slider_q, 1e-6),
                           slider_expected, 2e-7, 2e-7, "Prismatic difference");
        std::cout << "[PASS] Rotated prismatic axis\n";
    }

    /**
     * 空间差分测试调用：生成 7 活动关节 + 3 固定关节的专用测试链。
     * 类型、轴方向和安装朝向均有变化；固定连接在链首、中间和链尾。
     * 不是你的真实左臂，不读取 URDF，也不表示这些测试构型适合真机。
     */
    UrdfChain makeSpatialTestChain()
    {
        const std::array<UrdfJointType, 10> types{
            UrdfJointType::Fixed, UrdfJointType::Revolute,
            UrdfJointType::Prismatic, UrdfJointType::Continuous,
            UrdfJointType::Fixed, UrdfJointType::Revolute,
            UrdfJointType::Revolute, UrdfJointType::Prismatic,
            UrdfJointType::Revolute, UrdfJointType::Fixed};
        UrdfChain chain{"base", "tool", {}, {}};
        std::string parent = chain.base_link;
        for (std::size_t i = 0; i < types.size(); ++i)
        {
            const double s = static_cast<double>(i);
            UrdfChainJoint joint;
            joint.name = "joint_" + std::to_string(i);
            joint.parent_link = parent;
            joint.child_link = (i + 1 == types.size())
                                   ? chain.tip_link
                                   : "link_" + std::to_string(i);
            joint.type = types[i];
            joint.origin.position = Eigen::Vector3d{0.08, 0.01 * s, -0.03};
            const Eigen::Vector3d installation_axis =
                Eigen::Vector3d{0.2 + 0.03 * s, 1.0, -0.4}.normalized();
            joint.origin.orientation = Eigen::Quaterniond{
                Eigen::AngleAxisd{0.1 + 0.07 * s, installation_axis}};
            joint.axis = Eigen::Vector3d{1.0 + 0.1 * s, -0.3 + 0.07 * s, 0.4};
            if (joint.type != UrdfJointType::Fixed)
            {
                joint.q_index = chain.joint_names.size();
                chain.joint_names.push_back(joint.name);
            }
            chain.joints.push_back(joint);
            parent = joint.child_link;
        }
        return chain;
    }

    /** main 调用：同一空间模型上检查 200 个构型，每个使用两种差分步长。 */
    void testSpatialFiniteDifferences()
    {
        const ValidatedUrdfChain<7> model{makeSpatialTestChain()};
        std::mt19937 generator{20260916u};
        std::uniform_real_distribution<double> rotation{-3.2, 3.2};
        std::uniform_real_distribution<double> translation{-0.20, 0.20};
        std::array<double, 2> worst{0.0, 0.0};
        for (int sample = 0; sample < 200; ++sample)
        {
            std::array<double, 7> q{};
            if (sample != 0) // 第一组检查全零构型。
            {
                for (const auto &joint : model.chain().joints)
                {
                    if (!joint.q_index.has_value())
                        continue;
                    q[*joint.q_index] = (joint.type == UrdfJointType::Prismatic)
                                            ? translation(generator)
                                            : rotation(generator);
                }
            }
            // 连续关节在 pi 两侧也应通过，不能对包裹后的欧拉角直接做差。
            if (sample == 1)
                q[2] = kPi - 5e-7;
            if (sample == 2)
                q[2] = kPi + 5e-7;
            const auto actual = geometricJacobian(model, q);
            for (const double h : {1e-6, 2e-6})
            {
                const auto reference = finiteDifferenceJacobian(model, q, h);
                const auto errors = expectJacobianNear(
                    actual, reference, 2e-7, 2e-7,
                    "Spatial sample " + std::to_string(sample));
                worst[0] = std::max(worst[0], errors[0]);
                worst[1] = std::max(worst[1], errors[1]);
            }
        }
        std::cout << "[PASS] Spatial finite differences: 200 states, 2800 column checks\n"
                  << std::scientific << std::setprecision(3)
                  << "  Max linear-part error: " << worst[0] << '\n'
                  << "  Max angular-part error: " << worst[1] << '\n';
    }

    /** main 调用：检查非法 q，以及 FK 有限但雅可比运算溢出的拒绝路径。 */
    void testFailurePaths()
    {
        const ValidatedUrdfChain<2> model{makeTwoLinkChain()};
        const std::array<double, 3> invalid_values{
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
        for (const double invalid : invalid_values)
        {
            for (std::size_t i = 0; i < 2; ++i)
            {
                std::array<double, 2> q{0.0, 0.0};
                q[i] = invalid;
                expectThrows<std::invalid_argument>(
                    [&]
                    { (void)geometricJacobian(model, q); }, "Non-finite q");
            }
        }
        std::cout << "[PASS] Non-finite q rejection\n";

        // 纯数值边界：累计位置依次为 -M、0、+M，都有限；
        // 但第一关节到末端的位移 +M-(-M) 溢出。
        const double huge = std::numeric_limits<double>::max();
        UrdfChain source = makeTwoLinkChain();
        source.joints[0].origin.position.x() = -huge;
        source.joints[1].origin.position.x() = huge;
        source.joints[2].origin.position.x() = huge;
        const ValidatedUrdfChain<2> overflow_model{source};
        const std::array<double, 2> zero{0.0, 0.0};
        expectValidPose(forwardKinematics(overflow_model, zero));
        expectThrows<std::runtime_error>(
            [&]
            { (void)geometricJacobian(overflow_model, zero); }, "Jacobian overflow");
        std::cout << "[PASS] Jacobian-only overflow rejection\n";
    }
} // namespace

int main()
{
    try
    {
        testAnalyticCases();
        testSpatialFiniteDifferences();
        testFailurePaths();
        std::cout << "All geometric Jacobian tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Jacobian test failed: " << error.what() << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "Jacobian test failed: unknown exception\n";
        return 1;
    }
}