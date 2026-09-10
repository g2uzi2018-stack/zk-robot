#pragma once

#include "motion/geometry/pose.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace robot::motion
{

// 第一阶段只加载串联运动链的数据，不实现 FK / Jacobian / IK。
enum class UrdfJointType { Fixed, Revolute, Continuous, Prismatic };

struct UrdfChainJoint
{
    std::string name;
    std::string parent_link;
    std::string child_link;
    UrdfJointType type{UrdfJointType::Fixed};

    // URDF joint/origin：父连杆到零位关节坐标系的变换。
    // 不是 visual/origin 或 inertial/origin。长度 m，角度 rad。
    Pose origin;

    // 在关节局部坐标系表达的单位转轴 / 移动轴；fixed 不使用。
    Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};

    // 对应 q 中的下标。fixed 保留变换，但不占用 q 的一个元素。
    std::optional<std::size_t> q_index;
};

struct UrdfChain
{
    std::string base_link;
    std::string tip_link;
    std::vector<UrdfChainJoint> joints;       // base -> tip，含 fixed
    std::vector<std::string> joint_names;    // 仅活动关节，顺序对应 q
};

// 上游：初始化代码提供本地 URDF 文件、链的基准/末端连杆、预期活动关节数。
// 本函数：解析 XML，检查树结构，提取 base -> tip，保留固定变换并确定 q 顺序。
// 下游：后续具体 KinematicModel 保存返回数据，用于 FK / Jacobian。
//
// 只支持 base 是 tip 祖先的链。链上支持 fixed/revolute/continuous/prismatic，
// 拒绝 mimic/planar/floating/未知类型；不加载 mesh，不处理 xacro/DTD/实体。
// 这是运动学数据加载器，不验证完整 URDF 规范，不加载/执行限位或碰撞检查。
// 文件/格式/链不合法或活动关节数不匹配时抛 std::runtime_error，不返回半成品。
// 只在初始化时调用；不在控制周期中读文件，不连接 CAN 或驱动器。
UrdfChain loadUrdfChain(const std::string &urdf_path,
                       const std::string &base_link,
                       const std::string &tip_link,
                       std::size_t expected_joint_count);

} // namespace robot::motion
