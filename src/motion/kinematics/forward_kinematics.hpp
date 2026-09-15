#pragma once

#include "motion/geometry/pose.hpp"
#include "motion/kinematics/validated_urdf_chain.hpp"

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace robot::motion
{

// 上游：测试、规划程序或后续具体运动学模型。
//
// model：初始化阶段构造的已验证运动链，本函数只读使用。
// q：相对于模型几何零位的关节位置，不是本次运动增量。
//    顺序对应 model.chain().joint_names；旋转用 rad，移动用 m。
//    N 是活动关节数，固定关节不占用 q。
//
// 返回：tip_link 相对于 base_link 的当前位姿。
//       不隐式转换到整机 base_link，也不隐式附加掌心/TCP 偏移。
//
// 每次调用检查 q 与计算结果，不重复验证链拓扑、名称和下标，
// 不重复归一化模型中已验证的 origin 和 axis。
// 不读取文件、访问硬件、检查限位/碰撞，也不裁剪或修改 q。
//
// 失败约定：
//   std::invalid_argument：q 含 NaN 或无穷大。
//   std::runtime_error：计算产生无效位姿，例如平移运算溢出。
//   std::logic_error：已验证模型出现不支持的类型，表示内部约定被破坏。
// 失败时不返回部分结果；异常路径不作硬实时保证。
template <std::size_t N>
[[nodiscard]] Pose forwardKinematics(
    const ValidatedUrdfChain<N> &model,
    const std::array<double, N> &q)
{
    // q 每次都可能不同，不能因为模型已验证就跳过这项检查。
    for (std::size_t i = 0; i < N; ++i)
    {
        if (!std::isfinite(q[i]))
        {
            throw std::invalid_argument(
                "Joint position must be finite at q[" + std::to_string(i) + "]");
        }
    }

    const UrdfChain &chain = model.chain();

    // 开始时 current 就是 base：零平移、单位旋转。
    Pose base_from_current{};

    for (const UrdfChainJoint &joint : chain.joints)
    {
        // 先取零位安装关系，再叠加这一节的关节运动。
        // 只复制 Pose 数值，不复制关节名称等结构数据。
        Pose parent_from_child = joint.origin;

        switch (joint.type)
        {
        case UrdfJointType::Fixed:
            // 固定连接也参与累计，但不读取 q。
            break;

        case UrdfJointType::Revolute:
        case UrdfJointType::Continuous:
        {
            // 非固定关节的 q_index 存在且 < N，已由模型构造保证。
            const double joint_position = q[*joint.q_index];
            const Eigen::AngleAxisd angle_axis{joint_position, joint.axis};
            const Eigen::Quaterniond local_rotation{angle_axis};

            // 零位安装朝向 × 关节局部转动，不能交换次序。
            parent_from_child.orientation =
                joint.origin.orientation * local_rotation;
            // 旋转不改变本节子连杆原点在父系下的位置。
            break;
        }

        case UrdfJointType::Prismatic:
        {
            const double joint_position = q[*joint.q_index];
            const Eigen::Vector3d local_displacement =
                joint.axis * joint_position;

            // 局部移动量转到父连杆坐标系，再加零位安装位置。
            parent_from_child.position =
                joint.origin.position +
                joint.origin.orientation * local_displacement;
            // 移动关节不改变本节子连杆相对于父连杆的朝向。
            break;
        }

        default:
            throw std::logic_error(
                "Unsupported joint type in validated chain: " + joint.name);
        }

        // 本轮 current 是当前关节的 parent；连接关系由模型保证。
        // 使用单独的输出对象，避免提前覆盖累计朝向或位置。
        Pose base_from_child{};
        base_from_child.position =
            base_from_current.position +
            base_from_current.orientation * parent_from_child.position;
        base_from_child.orientation =
            base_from_current.orientation * parent_from_child.orientation;

        // 检查新算出的动态结果，并抑制累计四元数的范数漂移。
        // 这里不是重新检查模型中的静态安装位姿。
        try
        {
            base_from_current = normalizedPose(base_from_child);
        }
        catch (const std::invalid_argument &error)
        {
            throw std::runtime_error(
                "Invalid FK result at joint " + joint.name + ": " + error.what());
        }
    }

    // 模型已经保证遍历完所有关节后到达 tip_link。
    return base_from_current;
}

} // namespace robot::motion