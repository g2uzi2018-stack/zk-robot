#pragma once

#include "motion/geometry/pose.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>

namespace robot::motion
{

    // 一条固定自由度运动链的通用运动学接口。
    //
    // N:
    //   参与该运动链运动学计算的关节数量。
    //
    // 例如：
    //
    //   KinematicModel<7>
    //       表示一条 7 自由度运动链。
    //
    //   KinematicModel<6>
    //       表示一条 6 自由度运动链。
    //
    // KinematicModel 只描述数学运动学关系：
    //
    //       joint positions q
    //              ↓
    //       FK / Jacobian
    //              ↓
    //       end-effector motion
    //
    // 它不负责：
    //
    //   - 读取真实机器人
    //   - CAN
    //   - Controller / Executor
    //   - IK 迭代
    //   - DLS
    //   - MoveL
    //   - 轨迹时间参数化
    //   - joint limits
    //
    // 一个具体 KinematicModel 实例必须固定表示一条明确的运动链，
    // 并固定它自己的参考坐标系和末端坐标系。
    template <std::size_t N>
    class KinematicModel
    {
        static_assert(N > 0, "KinematicModel requires at least one joint");

    public:
        // 一组关节位置。
        //
        // 对普通旋转关节通常使用 rad。
        // 数组顺序必须与该运动学模型内部的关节顺序完全一致。
        using JointVector = std::array<double, N>;

        // 当前关节状态下的几何 Jacobian。
        //
        // 固定为 6 × N：
        //
        //   行 0~2：
        //       末端线运动部分 x / y / z
        //
        //   行 3~5：
        //       末端角运动部分 x / y / z
        //
        //   每一列：
        //       对应一个关节对末端运动的局部影响。
        using JacobianMatrix = Eigen::Matrix<double, 6, N>;

        static constexpr std::size_t kJointCount = N;

        virtual ~KinematicModel() = default;

        // 正运动学 FK。
        //
        // 输入：
        //
        //   q
        //       当前 N 个关节的位置。
        //
        // 作用：
        //
        //       根据运动链模型计算当前末端位姿。
        //
        // 输出：
        //
        //   Pose
        //       末端在该模型固定参考坐标系下的位置和姿态。
        virtual Pose forwardKinematics(
            const JointVector &q) const = 0;

        // Jacobian 函数 J(q)。
        //
        // 输入：
        //
        //   q
        //       当前 N 个关节的位置。
        //
        // 作用：
        //
        //       根据当前机械臂构型 q，
        //       计算“关节局部变化 → 末端局部变化”的映射关系。
        //
        // 输出：
        //
        //   6 × N Jacobian 矩阵。
        //
        // 注意：
        //
        //   Jacobian 不是固定矩阵。
        //
        //       q 改变
        //         ↓
        //       J(q) 通常也会改变
        //
        // Jacobian 的线运动和角运动部分必须表达在与
        // forwardKinematics() 返回 Pose 相同的参考坐标系下。
        virtual JacobianMatrix jacobian(
            const JointVector &q) const = 0;
    };

} // namespace robot::motion