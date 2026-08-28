#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace robot::motion
{

// 三维空间中的刚体位姿。
//
// position 的长度单位由调用链统一约定，机器人运动规划中应使用 m；
// orientation 表示目标局部坐标系相对于规划参考坐标系的姿态，即把目标局部
// 坐标系中的向量旋转到规划参考坐标系。
//
// Pose 只表示数学量，不携带 frame 名称。上游必须先把视觉、示教器等来源的
// 位姿转换到同一个规划坐标系，不能直接混用 world、base、tool 等坐标系。
struct Pose
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

// 检查位姿中的数值是否有限，并返回四元数归一化后的副本。
//
// 传感器或配置文件给出的四元数可能有很小的数值误差，因此合法的非单位
// 四元数在进入几何算法时统一归一化；零四元数没有旋转意义，必须拒绝。
inline Pose normalizedPose(const Pose &pose)
{
    if (!pose.position.allFinite() || !pose.orientation.coeffs().allFinite())
    {
        throw std::invalid_argument("Pose values must be finite");
    }

    const double quaternion_norm = pose.orientation.norm();
    if (!std::isfinite(quaternion_norm) || quaternion_norm <= 1e-12)
    {
        throw std::invalid_argument("Pose orientation must be non-zero");
    }

    Pose result = pose;
    result.orientation.normalize();
    return result;
}

} // namespace robot::motion
