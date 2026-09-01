#pragma once

#include "motion/geometry/pose.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot::motion
{

    // MoveL 使用的笛卡尔几何直线路径。
    //
    // progress 位于 [0, 1]：
    //   - position 沿起点到终点做直线插值；
    //   - orientation 沿两个姿态之间的最短旋转做四元数球面插值。
    //
    // 该类只回答“路径上 progress 处的末端位姿是什么”，不包含速度、持续时间、
    // IK、关节限制或线程。这样同一条几何路径以后可以配合不同的时间参数化算法。
    //
    // 上游：应用、视觉或任务逻辑提供已经转换到同一规划坐标系的 start/goal。
    // 下游：MoveL 规划器按 progress 采样，并以上一个关节解作为 seed 连续求 IK；
    //       得到关节路径后再做时间参数化，最终生成 JointTrajectory 交给 Executor。
    class CartesianLinePath
    {
    public:
        CartesianLinePath(Pose start, Pose goal) : start_(normalizedPose(start)), goal_(normalizedPose(goal))
        {
        }

        Pose sample(const double progress) const
        {
            if (!std::isfinite(progress))
            {
                throw std::invalid_argument(
                    "Cartesian path progress must be finite");
            }
            if (progress < 0.0 || progress > 1.0)
            {
                throw std::out_of_range(
                    "Cartesian path progress must be in [0, 1]");
            }

            Pose result;
            result.position =
                start_.position + progress * (goal_.position - start_.position);
            result.orientation =
                start_.orientation.slerp(progress, goal_.orientation).normalized();
            return result;
        }

        const Pose &start() const noexcept
        {
            return start_;
        }

        const Pose &goal() const noexcept
        {
            return goal_;
        }

        // 计算笛卡尔空间里，起点位置到终点位置的直线距离
        //  供后续 MoveL 根据最大笛卡尔采样间距决定 IK 采样数量。
        double translationLength() const noexcept
        {
            return (goal_.position - start_.position).norm();
        }

        // 从起始姿态旋转到目标姿态，最少需要转多少角度
        //  返回最短姿态变化角，范围为 [0, pi]，单位 rad。
        double rotationAngle() const noexcept
        {
            const double absolute_dot = std::abs(
                start_.orientation.dot(goal_.orientation));
            return 2.0 * std::acos(std::clamp(absolute_dot, 0.0, 1.0));
        }

    private:
        Pose start_;
        Pose goal_;
    };

} // namespace robot::motion
