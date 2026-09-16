#pragma once

#include "motion/geometry/pose.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace robot::motion
{

    /**
     * 当前末端位姿到目标末端位姿的误差，返回 6×1 向量，不是 Pose。
     * 上游：后续逆解迭代或测试。current 通常来自 FK(model, q)，
     * target 来自已经完成坐标换算的目标。参数顺序为 current、target。
     * 下游：求解器分别检查位置/角度误差，并用于计算下一次关节修正。
     *
     * 两个输入必须描述同一末端坐标系，并相对于同一固定参考坐标系。
     * 本工程中参考系为链 base，末端为链 tip；不自动附加 TCP 偏移。
     * Pose 没有 frame 名称，函数无法检查是否混用了 world/base/tool。
     * 本函数不读取 URDF、q 或限位，不修改输入，也不执行 IK。
     *
     * 定义（Rc、Rt 分别为当前与目标朝向对应的旋转矩阵）：
     *     e_position = target.position - current.position
     *     R_delta    = Rt * Rc^T，满足 R_delta * Rc = Rt
     *     e_rotation = axis(R_delta) * angle(R_delta)
     *     e          = [e_position; e_rotation]
     *
     * 行 0~2：在 base 中表达的位置差，单位 m。
     * 行 3~5：在 base 中表达的最短旋转向量，单位 rad；长度为旋转角。
     * 方向表示“当前如何转向目标”，不是 RPY 差、四元数系数差或角速度。
     * 这里的旋转轴是整体姿态误差的轴，不一定是机器人的任何一根关节轴。
     * 本定义是位置差 + SO(3) 旋转对数，不是完整 SE(3) 变换的对数。
     *
     * 数值与分支约定：
     *   输入沿用 normalizedPose() 的验证/归一化规则；位置不归一化。
     *   相对四元数 Q 与 -Q 表示相同旋转；优先选择 w >= 0 的表示。
     *   当浮点 w 恰为 0（半圈）时，使向量部绝对值最大分量为正；
     *   并列时由 Eigen 选择首个最大分量。这样为精确 Q/-Q 选定相同分支。
     *   半圈有两条等长旋转方向；这个约定不保证其邻域的方向连续，
     *   也不保证该方向满足关节约束或可达。此处不保存上一轮选轴状态。
     *   Eigen::AngleAxisd 处理零角及小角；零旋转返回零误差，不手动除以角度。
     *
     * 与几何雅可比的关系：
     *   e 的行顺序和表达坐标系与 J 一致，但 e 不是速度或关节增量。
     *   e_position.norm()、e_rotation.norm() 应使用各自容差；
     *   不把米和弧度未经权重直接混成一个“距离”。
     *   J 是速度雅可比，不是本误差关于 q 的精确导数：
     *   固定目标且接近零误差时，e(q+dq) ≈ e(q)-J(q)*dq。
     *   大姿态误差的严格线性化需要旋转对数的导数，求解器还需步长控制。
     *
     * 失败：无效输入由 normalizedPose() 抛出 invalid_argument；
     * 计算溢出/无效结果抛出 runtime_error，不返回伪装成成功的零向量。
     *
     * 例：位置从 (0.30,0.10,0.20) 到 (0.35,0.08,0.20)，
     * 朝向从单位旋转到绕 base Z 正转 90°：
     *     e = (0.05,-0.02,0, 0,0,pi/2)^T。
     */
    [[nodiscard]] inline Eigen::Matrix<double, 6, 1> poseErrorInBase(const Pose &current, const Pose &target)
    {
        /** 验证并复制输入；只归一化副本中的朝向。 */
        const Pose current_pose = normalizedPose(current);
        const Pose target_pose = normalizedPose(target);

        Eigen::Matrix<double, 6, 1> error = Eigen::Matrix<double, 6, 1>::Zero();
        error.head<3>() = target_pose.position - current_pose.position;
        if (!error.head<3>().allFinite())
        {
            throw std::runtime_error("Pose position error overflow");
        }

        /** 单位四元数共轭等于逆；左乘修正，故旋转误差表达在 base 中。 */
        Eigen::Quaterniond delta_rotation =
            target_pose.orientation * current_pose.orientation.conjugate();
        const double delta_norm = delta_rotation.norm();
        if (!delta_rotation.coeffs().allFinite() ||
            !std::isfinite(delta_norm) || delta_norm <= 1e-12)
        {
            throw std::runtime_error("Invalid relative rotation in pose error");
        }
        delta_rotation.normalize();

        /** 只改变四元数表示的符号，不改变它代表的旋转。 */
        if (delta_rotation.w() < 0.0)
        {
            delta_rotation.coeffs() *= -1.0;
        }
        if (delta_rotation.w() == 0.0)
        {
            Eigen::Index dominant = 0;
            delta_rotation.vec().cwiseAbs().maxCoeff(&dominant);
            if (delta_rotation.vec()[dominant] < 0.0)
            {
                delta_rotation.coeffs() *= -1.0;
            }
        }

        /** 四元数 -> 角轴 -> 三维旋转向量；角度在 [0, pi] 内。 */
        const Eigen::AngleAxisd angle_axis{delta_rotation};
        error.tail<3>() = angle_axis.axis() * angle_axis.angle();
        if (!error.allFinite())
        {
            throw std::runtime_error("Non-finite pose error");
        }
        return error;
    }

} // namespace robot::motion