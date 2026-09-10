#pragma once

#include "motion/geometry/pose.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace robot::motion
{

// 第一版准备支持的关节类型。
enum class UrdfJointType
{
    Fixed,       // 固定连接，不运动。
    Revolute,    // 有角度范围的旋转关节。
    Continuous,  // 不设角度上下界的连续旋转关节。
    Prismatic    // 沿指定轴直线移动的关节。
};

// 运动链中的一个关节：只保存结构数据，不保存当前关节角。
struct UrdfChainJoint
{
    std::string name;

    // 这个关节连接哪两个连杆。
    std::string parent_link;
    std::string child_link;

    UrdfJointType type{UrdfJointType::Fixed};

    // URDF 中 joint/origin 描述的固定安装变换：
    // 零位关节坐标系相对于父连杆坐标系的位置和朝向。
    // position 使用 m；orientation 使用四元数。
    Pose origin{};

    // 在关节局部坐标系下表达的单位轴。
    // 旋转关节表示转轴，移动关节表示移动方向。
    // 固定关节不使用这个字段。
    Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};

    // 这个关节对应输入数组 q 的哪个元素。
    // 固定关节不占用 q，因此用 nullopt 表示“没有下标”。
    std::optional<std::size_t> q_index{std::nullopt};
};

// 从指定基准连杆到指定末端连杆的一整条运动链。
struct UrdfChain
{
    std::string base_link;
    std::string tip_link;

    // 按 base -> tip 排列，包含固定关节。
    std::vector<UrdfChainJoint> joints;

    // 只保存活动关节的名字，排列顺序对应输入数组 q。
    std::vector<std::string> joint_names;
};

// 上游：程序初始化代码，提供 URDF 文件与运动链选择参数。
//
// 本函数的职责：
//   读取 URDF，提取 base_link -> tip_link 的运动链，
//   保存结构数据，并确定活动关节在 q 中的排列顺序。
//
// 下游：运动学模型保存返回的 UrdfChain，供后续正解和雅可比计算使用。
//
// 接口约定：
//   文件、运动链或活动关节数不合法时，抛出 std::runtime_error。
//   只在初始化阶段读取文件，不在控制周期中反复加载。
//   不负责机器人通信、关节限位执行、碰撞检测或逆运动学。
//
// 此处只有声明，具体实现放在后续的 urdf_chain.cpp 中。
UrdfChain loadUrdfChain(
    const std::string &urdf_path,
    const std::string &base_link,
    const std::string &tip_link,
    std::size_t expected_joint_count);

} // namespace robot::motion