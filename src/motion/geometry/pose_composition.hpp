#pragma once

#include "motion/geometry/pose.hpp"

#include <Eigen/Core>

namespace robot::motion
{

    // 组合两段首尾相接的位姿关系。
    //
    // 上游：整条运动链的正解函数，或纯数学测试程序。
    //
    // 输入：
    //   base_from_parent：父连杆相对于基准坐标系的当前位姿。
    //   parent_from_child：子连杆相对于该父连杆的当前位姿。
    //
    // 返回：
    //   子连杆相对于基准坐标系的当前位姿。
    //
    // 注意：
    //   两个参数中的 parent 必须是同一个坐标系。
    //   Pose 不保存坐标系名称，因此调用者必须保证这个对应关系。
    //   本函数不读取 URDF，不计算关节转角，也不访问机器人。
    //
    // 下游：正解函数继续连接下一段，或将最终位姿交给调用者。
    inline Pose composePoses(
        const Pose &base_from_parent,
        const Pose &parent_from_child)
    {
        // 校验输入，并归一化姿态四元数。
        const Pose parent_pose = normalizedPose(base_from_parent);
        const Pose child_pose = normalizedPose(parent_from_child);

        // 子连杆原点相对于父连杆的位移，
        // 原本用父连杆坐标表达，现在换算为基准坐标表达。
        const Eigen::Vector3d displacement_in_base =
            parent_pose.orientation * child_pose.position;

        Pose result;

        // 父连杆原点的位置 + 换算后的父到子位移。
        result.position =
            parent_pose.position + displacement_in_base;

        // 组合两段朝向，顺序不能交换。
        result.orientation =
            parent_pose.orientation * child_pose.orientation;

        // 检查结果是否合法，并归一化结果四元数。
        return normalizedPose(result);
    }

} // namespace robot::motion