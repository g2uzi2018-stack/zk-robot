#pragma once

#include "motion/kinematics/urdf_chain.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>
#include <string>

namespace robot::motion
{

    // 计算一个关节处，子连杆相对于父连杆的当前位姿。
    //
    // 上游：整条链的正解函数，或纯数学测试程序。
    //   joint：已经加载好的关节结构数据。
    //   joint_position：这个关节的位置，不是整组 q。
    //     revolute / continuous 使用 rad；prismatic 使用 m。
    //     fixed 没有活动位置，本接口约定必须传入 0.0。
    //
    // 返回值：
    //   子连杆坐标系相对于父连杆坐标系的 Pose。
    //   同一点的坐标换算：p_parent = orientation * p_child + position。
    //
    // 下游：整条链的正解函数，将相邻连杆的变换逐级组合。
    // 不读取 URDF、不访问硬件、不检查运动限位或碰撞。
    inline Pose jointTransform(const UrdfChainJoint &joint, const double joint_position) // joint_position是几何上的相对几何零点的位置,而不是每次转动的增量
    {
        if (!std::isfinite(joint_position))
        {
            throw std::invalid_argument("Joint position must be finite: " + joint.name);
        }

        // 起点是 URDF 中保存的零位安装关系。这个关节相对于父节点的安装位姿
        const Pose origin = normalizedPose(joint.origin);

        // 固定关节只有安装关系，没有额外运动。
        if (joint.type == UrdfJointType::Fixed)
        {
            if (joint_position != 0.0)
            {
                throw std::invalid_argument("Fixed joint position must be zero: " + joint.name);
            }
            return origin;
        }

        // 保护数学入口：活动关节的局部轴必须有效。
        if (!joint.axis.allFinite())
        {
            throw std::invalid_argument("Joint axis must be finite: " + joint.name);
        }
        // 计算轴的范数 开根号(x^2 + y^2 + z^2)
        const double axis_norm = joint.axis.stableNorm();
        if (!std::isfinite(axis_norm) || axis_norm <= 1e-12)
        {
            throw std::invalid_argument("Joint axis must be non-zero: " + joint.name);
        }
        // 正则化轴
        const Eigen::Vector3d axis = joint.axis / axis_norm;

        Pose result = origin;

        switch (joint.type)
        {
        case UrdfJointType::Revolute: // 利用switch的特性 Revolute Continuous 使用同一种类的逻辑
        case UrdfJointType::Continuous:
        {
            // 在关节局部坐标系中，绕 axis 旋转 joint_position。
            // 用“角度 + 转轴”描述这次局部旋转。 AngleAxisd angle_axis{joint_position, axis};
            // 把同一个旋转转换成四元数表示。const Eigen::Quaterniond rotation{angle_axis};
            const Eigen::Quaterniond rotation{Eigen::AngleAxisd{joint_position, axis}};

            // 安装朝向 × 局部转动。不能交换相乘顺序 
            // 这里的 * 是 Eigen 定义的旋转组合运算，不是把两个四元数的四个分量逐项相乘这里的 * 是 Eigen 定义的旋转组合运算，不是把两个四元数的四个分量逐项相乘
            /**
             * | 变量                   | 含义                       |
                | -------------------- | ------------------------ |
                | `origin.orientation` | 零位安装朝向：关节零位参考系相对于父连杆怎样摆放 |
                | `rotation`           | 关节局部转动：子连杆相对于零位转了多少      |
                | `result.orientation` | 组合后的结果：当前子连杆相对于父连杆的朝向    |

             */
            result.orientation = origin.orientation * rotation;

            // 这种旋转关节的子连杆原点位于关节原点，
            // 平移保持 origin.position。
            break;
        }

        case UrdfJointType::Prismatic:
            // 先算局部移动量，再转到父连杆坐标系，
            // 最后加上安装位置。
            result.position = origin.position +
                              origin.orientation * (axis * joint_position);
            break;

        default:
            throw std::invalid_argument(
                "Unsupported joint type: " + joint.name);
        }

        // 检查计算结果，归一化姿态；同时拒绝平移运算溢出。
        return normalizedPose(result);
    }

} // namespace robot::motion