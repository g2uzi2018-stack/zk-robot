#include "motion/kinematics/forward_kinematics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstddef>
#include <exception>
#include <iomanip>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
using namespace robot::motion;

const double kPi = std::acos(-1.0);
constexpr double kPositionTolerance = 1e-9;  // m，仅为本数学测试的容差。
constexpr double kAngleTolerance = 1e-9;     // rad，不是实际机械臂精度指标。
constexpr double kQuaternionNormTolerance = 1e-12;

// 各测试调用：比较实际位姿和独立给定的预期位姿。
// 不归一化实际输出，以免替被测函数掩盖错误。
void expectPoseNear(
    const Pose &actual,
    const Pose &expected,
    const std::string &message)
{
    const auto is_valid = [](const Pose &pose)
    {
        const double norm = pose.orientation.norm();
        return pose.position.allFinite() &&
               pose.orientation.coeffs().allFinite() &&
               std::isfinite(norm) &&
               std::abs(norm - 1.0) <= kQuaternionNormTolerance;
    };

    if (!is_valid(actual) || !is_valid(expected))
    {
        throw std::runtime_error(message + ": invalid/non-unit pose");
    }

    const double position_error = (actual.position - expected.position).norm();
    const double angle_error = actual.orientation.angularDistance(expected.orientation);

    if (!std::isfinite(position_error) || !std::isfinite(angle_error) ||
        position_error > kPositionTolerance || angle_error > kAngleTolerance)
    {
        std::cerr << std::scientific << std::setprecision(12)
                  << message << '\n'
                  << "  Position error (m): " << position_error << '\n'
                  << "  Rotation error (rad): " << angle_error << '\n';
        throw std::runtime_error(message);
    }
}

// 各失败路径测试调用：执行 action，要求它抛出指定类别的异常。
// 没抛异常、抛了其他类别异常，都属于测试失败。
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
        throw std::runtime_error(message + ": unexpected exception: " + error.what());
    }
    throw std::runtime_error(message + ": expected exception was not thrown");
}

// 测试数据：两活动关节 + 一固定末端偏移，不读取 URDF 文件。
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

void testTwoLink()
{
    UrdfChain source = makeTwoLinkChain();
    const ValidatedUrdfChain<2> model{source};
    const std::array<double, 2> zero{0.0, 0.0};

    Pose expected_zero{};
    expected_zero.position = Eigen::Vector3d{0.50, 0.0, 0.0};
    expectPoseNear(forwardKinematics(model, zero), expected_zero, "Zero pose");

    const std::array<double, 2> q{kPi / 6.0, kPi / 3.0};
    const auto q_before = q;
    Pose expected{};
    // 手算：x = 0.30*cos(30 deg)，y = 0.30*sin(30 deg) + 0.20。
    expected.position = Eigen::Vector3d{0.15 * std::sqrt(3.0), 0.35, 0.0};
    expected.orientation = Eigen::Quaterniond{
        Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};

    const Pose actual = forwardKinematics(model, q);
    expectPoseNear(actual, expected, "Two-link pose");
    std::cout << std::fixed << std::setprecision(6)
              << "[PASS] Two-link chain: "
              << actual.position.x() << ' ' << actual.position.y() << ' '
              << actual.position.z() << '\n';

    // 重用同一模型：q 是位置，不是累加到上次结果上的增量。
    expectPoseNear(forwardKinematics(model, zero), expected_zero, "Return to zero");
    // 改原始链，不能改变模型已经独立保存的几何数据。
    source.joints[1].origin.position.x() = 99.0;
    expectPoseNear(forwardKinematics(model, q), expected, "Source isolation");
    if (q != q_before)
    {
        throw std::runtime_error("FK modified its input q");
    }
    std::cout << "[PASS] Model reuse and source isolation\n";

    // continuous 同样走新版入口；超过一圈的 q 不改变预期几何姿态。
    UrdfChain continuous_source = makeTwoLinkChain();
    continuous_source.joints[0].type = UrdfJointType::Continuous;
    const ValidatedUrdfChain<2> continuous_model{continuous_source};
    const std::array<double, 2> continuous_q{2.0 * kPi + kPi / 6.0, kPi / 3.0};
    expectPoseNear(forwardKinematics(continuous_model, continuous_q),
                   expected, "Continuous joint pose");
    std::cout << "[PASS] Continuous rotation\n";
}

void testRotationOrder()
{
    UrdfChainJoint joint;
    joint.name = "rotated_joint";
    joint.parent_link = "base";
    joint.child_link = "rotated_link";
    joint.type = UrdfJointType::Revolute;
    joint.origin.position = Eigen::Vector3d{0.30, 0.10, 0.0};
    joint.origin.orientation = Eigen::Quaterniond{
        Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
    joint.axis = Eigen::Vector3d::UnitX();
    joint.q_index = 0;

    UrdfChain source{"base", "rotated_link", {joint}, {joint.name}};
    const ValidatedUrdfChain<1> model{source};
    const std::array<double, 1> q{kPi / 2.0};
    Pose expected{};
    expected.position = Eigen::Vector3d{0.30, 0.10, 0.0};
    // 手算 Rz(90 deg)*Rx(90 deg)：X->Y、Y->Z、Z->X。
    // Eigen 四元数构造参数顺序为 w、x、y、z。
    expected.orientation = Eigen::Quaterniond{0.5, 0.5, 0.5, 0.5};
    expectPoseNear(forwardKinematics(model, q), expected, "Joint rotation order");
    std::cout << "[PASS] Rotation order\n";

    // 再接一段带旋转的固定变换，单独覆盖累计朝向的乘法顺序。
    UrdfChainJoint tool;
    tool.name = "rotated_tool_mount";
    tool.parent_link = "rotated_link";
    tool.child_link = "tool";
    tool.type = UrdfJointType::Fixed;
    tool.origin.position = Eigen::Vector3d{0.20, 0.10, 0.05};
    tool.origin.orientation = Eigen::Quaterniond{
        Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
    source.joints.push_back(tool);
    source.tip_link = "tool";
    const ValidatedUrdfChain<1> tool_model{source};

    // 手算：偏移变为 (0.05, 0.20, 0.10)，再加 (0.30, 0.10, 0)。
    expected.position = Eigen::Vector3d{0.35, 0.30, 0.10};
    // 最终 X->Z、Y->-Y、Z->X，即绕 (X+Z)/sqrt(2) 旋转 pi。
    const double half_sqrt_two = std::sqrt(0.5);
    expected.orientation = Eigen::Quaterniond{0.0, half_sqrt_two, 0.0, half_sqrt_two};
    expectPoseNear(forwardKinematics(tool_model, q), expected, "Fixed tool transform");
    std::cout << "[PASS] Fixed tool transform\n";
}

void testPrismatic()
{
    UrdfChainJoint slider;
    slider.name = "slider";
    slider.parent_link = "base";
    slider.child_link = "carriage";
    slider.type = UrdfJointType::Prismatic;
    slider.origin.position = Eigen::Vector3d{0.30, 0.10, 0.0};
    slider.origin.orientation = Eigen::Quaterniond{
        Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
    slider.q_index = 0;

    // 故意使用非单位轴和非单位四元数，检查初始化归一化的接入。
    // position 仍应保持真实距离，不能被归一化。
    slider.axis = Eigen::Vector3d{2.0, 0.0, 0.0};
    slider.origin.orientation.coeffs() *= 2.0;
    const UrdfChain source{"base", "carriage", {slider}, {slider.name}};
    const ValidatedUrdfChain<1> model{source};

    Pose expected{};
    expected.orientation = Eigen::Quaterniond{
        Eigen::AngleAxisd{kPi / 2.0, Eigen::Vector3d::UnitZ()}};
    for (const double displacement : {0.20, -0.20, 0.0})
    {
        // 安装时绕 Z 转 90 deg：局部 +X 移动对应基准 +Y。
        expected.position = Eigen::Vector3d{0.30, 0.10 + displacement, 0.0};
        expectPoseNear(forwardKinematics(model, std::array<double, 1>{displacement}),
                       expected, "Prismatic pose");
    }
    std::cout << "[PASS] Prismatic displacement and normalization\n";
}

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
                [&] { (void)forwardKinematics(model, q); }, "Non-finite q");
        }
    }
    std::cout << "[PASS] Non-finite q rejection\n";

    // 有限输入也可能在计算中溢出；这不是有实际机械尺寸的测试模型。
    const double huge = std::numeric_limits<double>::max();
    UrdfChainJoint slider;
    slider.name = "overflow_slider";
    slider.parent_link = "base";
    slider.child_link = "carriage";
    slider.type = UrdfJointType::Prismatic;
    slider.axis = Eigen::Vector3d::UnitX();
    slider.q_index = 0;
    slider.origin.position = Eigen::Vector3d{huge, 0.0, 0.0};
    const UrdfChain source{"base", "carriage", {slider}, {slider.name}};
    const ValidatedUrdfChain<1> overflow_model{source};
    expectThrows<std::runtime_error>(
        [&] { (void)forwardKinematics(overflow_model, std::array<double, 1>{huge}); },
        "Arithmetic overflow");
    std::cout << "[PASS] Arithmetic overflow rejection\n";

    // 初始化必须拒绝坏模型，不能让错误下标进入不重复检查下标的 FK。
    UrdfChain broken = makeTwoLinkChain();
    broken.joints[1].q_index = 2;  // 两自由度模型只有 q[0]、q[1]。
    expectThrows<std::invalid_argument>(
        [&] { const ValidatedUrdfChain<2> rejected{broken}; (void)rejected; },
        "Out-of-range q_index");
    broken = makeTwoLinkChain();
    broken.joints[1].parent_link = "unrelated_link";
    expectThrows<std::invalid_argument>(
        [&] { const ValidatedUrdfChain<2> rejected{broken}; (void)rejected; },
        "Broken chain");
    broken = makeTwoLinkChain();
    broken.joints[0].axis.setZero();
    expectThrows<std::invalid_argument>(
        [&] { const ValidatedUrdfChain<2> rejected{broken}; (void)rejected; },
        "Zero axis");
    std::cout << "[PASS] Invalid model rejection\n";
}

} // namespace

int main()
{
    try
    {
        testTwoLink();
        testRotationOrder();
        testPrismatic();
        testFailurePaths();
        std::cout << "All FK interface tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "FK interface test failed: " << error.what() << '\n';
        return 1;
    }
    catch (...)
    {
        std::cerr << "FK interface test failed: unknown exception\n";
        return 1;
    }
}