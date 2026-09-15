#include "motion/geometry/pose_composition.hpp"
#include "motion/kinematics/urdf_joint_transform.hpp"

#include <array>
#include <cmath>
#include <exception>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

    // 我们自己的测试辅助函数：检查两个向量是否足够接近。
    void expectVectorNear(
        const Eigen::Vector3d &actual,
        const Eigen::Vector3d &expected,
        const std::string &message)
    {
        const double error = (actual - expected).norm();

        if (!std::isfinite(error) || error > 1e-9)
        {
            std::cerr
                << message << '\n'
                << "  Actual: " << actual.x() << ' '
                << actual.y() << ' ' << actual.z() << '\n'
                << "  Expected: " << expected.x() << ' '
                << expected.y() << ' ' << expected.z() << '\n';

            throw std::runtime_error(message);
        }
    }

} // namespace

int main()
{
    try
    {
        using namespace robot::motion;
        const double pi = std::acos(-1.0);

        // 测试一：两轴平面手臂，上臂 0.30 m，小臂 0.20 m。
        // 零位时两段都沿 +X 伸直，两个旋转轴都指向局部 +Z。
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

        // 手的位置选在小臂末端，需要保留这段固定偏移。
        UrdfChainJoint tool;
        tool.name = "tool_mount";
        tool.parent_link = "forearm";
        tool.child_link = "tool";
        tool.type = UrdfJointType::Fixed;
        tool.origin.position = Eigen::Vector3d{0.20, 0.0, 0.0};

        UrdfChain chain;
        chain.base_link = "base";
        chain.tip_link = "tool";
        chain.joints = {shoulder, elbow, tool};
        chain.joint_names = {"shoulder", "elbow"};

        // 肩相对零位转 30°，肘相对自己的零位转 60°。
        const std::array<double, 2> q{pi / 6.0, pi / 3.0};

        // 基准连杆相对于自身：位置为零，朝向为单位旋转。
        Pose base_from_current{};

        for (const auto &joint : chain.joints)
        {
            double joint_position = 0.0;
            if (joint.q_index.has_value())
            {
                joint_position = q.at(*joint.q_index);
            }

            const Pose parent_from_child =
                jointTransform(joint, joint_position);

            base_from_current =
                composePoses(base_from_current, parent_from_child);
        }

        // 独立手算：手的位置 = (0.30*cos30°, 0.30*sin30°+0.20, 0)。
        expectVectorNear(
            base_from_current.position,
            Eigen::Vector3d{0.15 * std::sqrt(3.0), 0.35, 0.0},
            "Two-link tool position mismatch");

        // 总朝向是绕 Z 转 90°：手的 X 指向基准 Y，Z 方向不变。
        expectVectorNear(
            base_from_current.orientation * Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY(),
            "Two-link tool X direction mismatch");
        expectVectorNear(
            base_from_current.orientation * Eigen::Vector3d::UnitZ(),
            Eigen::Vector3d::UnitZ(),
            "Two-link tool Z direction mismatch");

        std::cout << std::fixed << std::setprecision(6)
                  << "[PASS] Two-link chain: "
                  << base_from_current.position.x() << ' '
                  << base_from_current.position.y() << ' '
                  << base_from_current.position.z() << '\n';

        // 测试二：安装时绕 Z 转 90°，关节再绕自己的 X 转 90°。
        // 这两个不同轴的旋转能检查乘法顺序是否写反。
        UrdfChainJoint rotated_joint;
        rotated_joint.name = "rotated_joint";
        rotated_joint.type = UrdfJointType::Revolute;
        rotated_joint.origin.orientation = Eigen::Quaterniond{
            Eigen::AngleAxisd{pi / 2.0, Eigen::Vector3d::UnitZ()}};
        rotated_joint.axis = Eigen::Vector3d::UnitX();

        const Pose rotated_pose = jointTransform(rotated_joint, pi / 2.0);
        expectVectorNear(
            rotated_pose.orientation * Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY(),
            "Rotation order: X direction mismatch");
        expectVectorNear(
            rotated_pose.orientation * Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ(),
            "Rotation order: Y direction mismatch");

        // 同样的两段旋转，交给 composePoses()，也必须保持相同顺序。
        Pose local_rotation{};
        local_rotation.orientation = Eigen::Quaterniond{
            Eigen::AngleAxisd{pi / 2.0, Eigen::Vector3d::UnitX()}};
        const Pose composed_pose =
            composePoses(rotated_joint.origin, local_rotation);
        expectVectorNear(
            composed_pose.orientation * Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY(),
            "Composition order: X direction mismatch");
        expectVectorNear(
            composed_pose.orientation * Eigen::Vector3d::UnitY(),
            Eigen::Vector3d::UnitZ(),
            "Composition order: Y direction mismatch");
        std::cout << "[PASS] Rotation order\n";

        // 测试三：安装朝向不为零时，移动量必须先换到父坐标系。
        UrdfChainJoint slider = rotated_joint;
        slider.name = "slider";
        slider.type = UrdfJointType::Prismatic;
        slider.origin.position = Eigen::Vector3d{0.30, 0.10, 0.0};

        const Pose slider_pose = jointTransform(slider, 0.20);
        expectVectorNear(
            slider_pose.position,
            Eigen::Vector3d{0.30, 0.30, 0.0},
            "Prismatic position mismatch");
        expectVectorNear(
            slider_pose.orientation * Eigen::Vector3d::UnitX(),
            Eigen::Vector3d::UnitY(),
            "Prismatic orientation mismatch");
        std::cout << "[PASS] Prismatic displacement\n";

        std::cout << "Kinematics math tests passed.\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Kinematics math test failed: " << error.what() << '\n';
        return 1;
    }
}