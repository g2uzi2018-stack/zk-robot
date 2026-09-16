#pragma once

#include "motion/kinematics/forward_kinematics.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>

/**
 * 几何雅可比矩阵：含义、尺寸与计算公式
 *
 * 一、它解决什么问题？
 *
 * 正运动学 FK：
 *     已知各个关节的位置 q，计算末端现在的位置和朝向。
 *
 * 几何雅可比 J(q)：
 *     已知当前构型 q，计算“关节速度 -> 末端速度”的映射关系。
 *
 * 注意：
 *     本函数输入的是关节位置 q，不是关节速度。
 *     因为必须先知道机械臂当前怎样摆放，才能确定各个关节对末端的影响。
 *     本函数返回矩阵 J(q)，并不直接返回末端速度。
 *     调用者再提供关节速度 q_dot，才能计算实际末端速度。
 *
 *
 * 二、整体关系与矩阵尺寸
 *
 *     [ v_tip_in_base     ] = J(q) * q_dot
 *     [ omega_tip_in_base ]
 *
 * 尺寸：
 *
 *            6×1           = 6×N  *  N×1
 *
 * 其中：
 *
 *     q：
 *         当前各活动关节相对于模型几何零位的位置，不是运动增量。
 *         旋转关节使用 rad，移动关节使用 m。
 *
 *     q_dot：
 *         各活动关节的速度，顺序与 q 完全一致。
 *         旋转关节使用 rad/s，移动关节使用 m/s。
 *
 *     v_tip_in_base：
 *         末端原点相对于链基准的线速度，用基准坐标系的坐标轴表达。
 *         是 3×1 向量，分量为 (vx, vy, vz)，单位 m/s。
 *
 *     omega_tip_in_base：
 *         末端连杆相对于链基准的角速度，用基准坐标系的坐标轴表达。
 *         是 3×1 向量，分量为 (wx, wy, wz)，单位 rad/s。
 *         omega 表示角速度，不是当前朝向，也不是 RPY 三个角的导数。
 *         这里的 wx、wy、wz 也不是四元数的分量。
 *
 *     [v; omega]：
 *         将两个 3×1 向量上下拼接，得到 6×1 的末端速度向量。
 *         分号表示上下拼接，不是相加。
 *         这个 6×1 向量不是雅可比矩阵；J(q) 才是雅可比矩阵。
 *
 *     N：
 *         q 中独立关节位置变量的数量。
 *         当前实现中，每个活动关节只有一个自由度，
 *         所以 N 等于活动关节数，不包括固定关节。
 *         例如：7 个活动关节加若干固定关节，J 仍然是 6×7。
 *
 *
 * 三、一个 J_i 是什么？
 *
 * J_i 是完整雅可比矩阵的第 i 列，尺寸是 6×1：
 *
 *             [ Jv_i     ]   上半部分为 3×1，表示线运动贡献
 *     J_i  =  [ Jomega_i ]   下半部分为 3×1，表示角运动贡献
 *
 * 完整矩阵由这些列并排组成：
 *
 *     J(q) = [ J_0  J_1  ...  J_(N-1) ]
 *
 *             每列 6×1，共 N 列，因此整体为 6×N。
 *
 * 行与列的对应关系：
 *
 *     行 0、1、2：线速度的 X、Y、Z 分量。
 *     行 3、4、5：角速度的 X、Y、Z 分量。
 *     第 i 列：对应 q[i] 所代表的那个活动关节。
 *
 * 单独看第 i 个关节：
 *
 *     J_i * q_dot[i]
 *
 * 得到这个关节对末端速度的 6×1 贡献。
 * J_i 是一个列向量，q_dot[i] 是一个标量。
 *
 * 所有关节的贡献相加：
 *
 *     [v; omega] = J_0*q_dot[0] + J_1*q_dot[1] + ...
 *
 * 这与矩阵乘法 J(q) * q_dot 是同一件事。
 * 所有列都描述对“同一个总末端”的影响，
 * 不是每一列分别描述一个不同连杆末端的速度。
 *
 *
 * 四、计算一列所需的三个几何量
 *
 * 以下三个量全部用链基准坐标系表达，不能混用局部坐标。
 *
 * 1. a_i：第 i 个关节的单位轴方向，3×1 向量。
 *
 *     代码对应：
 *         axis_in_base
 *
 *     例如 (0, 0, 1)，表示轴朝向基准 Z 正方向。
 *     它只有方向，没有位置，也不包含完整的坐标系朝向。
 *     它不是标量，也不是 Pose。
 *
 *     来源：
 *         a_i = 当前关节参考系的基准朝向 * joint.axis
 *
 *     joint.axis 来自 URDF 的 <axis xyz="...">，
 *     原本表达在关节局部坐标系中，必须先旋转到基准坐标系。
 *
 * 2. o_i：第 i 个关节轴上选取的原点位置，3×1 向量，单位 m。
 *
 *     代码对应：
 *         base_from_joint.position
 *
 *     它只表示这个点在基准坐标系中的位置，不包含朝向。
 *     它不是整个 base_from_joint，也不是 Pose。
 *
 *     本实现中：
 *         先累计上游关节的当前运动，再接上当前 joint/origin，
 *         取得当前关节尚未施加自身运动时的参考系原点。
 *         这不意味着整条机械臂处于零位。
 *
 * 3. p：整条链末端原点的位置，3×1 向量，单位 m。
 *
 *     代码对应：
 *         base_from_tip.position
 *
 *     它由整条链的正运动学计算得到，所有雅可比列使用同一个 p。
 *     它只包含位置，不包含末端朝向。
 *
 *     不能一般性地用“o_i + a_i * 连杆长度”得到 p：
 *         转轴方向不一定是连杆伸出的方向；
 *         p 还取决于各关节位置、安装关系以及末端固定偏移。
 *
 * 三者的直观含义：
 *
 *     a_i：轴朝哪里。
 *     o_i：轴经过哪里。
 *     p  ：总末端现在在哪里。
 *
 * 类型区别：
 *
 *     a_i、o_i、p：都是 Eigen::Vector3d。
 *     Pose       ：同时包含 position 和 orientation。
 *
 *     a_i 和 o_i 合在一起可以确定一根有方向的空间轴线，
 *     但不等于一个连杆的完整位姿。
 *
 *
 * 五、旋转关节的雅可比列
 *
 * revolute 和 continuous 使用相同的几何公式：
 *
 *             [ a_i × (p - o_i) ]
 *     J_i  =  [       a_i       ]       尺寸为 6×1
 *
 * 即：
 *
 *     Jv_i     = a_i × (p - o_i)         3×1
 *     Jomega_i = a_i                     3×1
 *
 * 其中：
 *
 *     p - o_i：
 *         从关节轴原点指向总末端的位移向量。
 *
 *     ×：
 *         三维向量叉乘，代码中为 axis_in_base.cross(joint_to_tip)。
 *         不是点积，也不是逐分量相乘。
 *         交换叉乘两边的顺序会改变符号。
 *
 * 原因：
 *
 *     绕轴运动的瞬时线速度满足 v = omega × r。
 *     当前关节产生的角速度为 omega = a_i * q_dot[i]。
 *     因此线速度贡献为：
 *
 *         v_i = [a_i × (p - o_i)] * q_dot[i]
 *
 *     角速度贡献为：
 *
 *         omega_i = a_i * q_dot[i]
 *
 * 简单例子：
 *
 *     a_i = (0, 0, 1)
 *     p - o_i = (0.3, 0, 0) m
 *
 *     则 J_i 为：
 *
 *         [ 0   ]
 *         [ 0.3 ]
 *         [ 0   ]
 *         [ 0   ]
 *         [ 0   ]
 *         [ 1   ]
 *
 *     当该关节速度为 1 rad/s、其他关节速度为零时：
 *         末端线速度为 (0, 0.3, 0) m/s；
 *         末端角速度为 (0, 0, 1) rad/s。
 *
 *
 * 六、移动关节与固定关节
 *
 * prismatic：
 *
 *             [ a_i ]
 *     J_i  =  [  0  ]       这里下方的 0 是 3×1 零向量。
 *
 *     即 Jv_i = a_i，Jomega_i = (0, 0, 0)。
 *     沿轴移动产生平移贡献，不由这个关节本身产生角速度。
 *
 * fixed：
 *
 *     不占用 q，不产生雅可比列。
 *     但固定关节仍然参与位姿累计。
 *     固定偏移会改变总末端位置，从而影响前面旋转关节的线运动贡献。
 *
 *
 * 七、末端、参考坐标系与使用边界
 *
 * tip_link：
 *     选定的末端连杆名称；link 是连杆，joint 才是关节。
 *     本函数针对 tip_link 坐标系的原点，不是模型外形上最远的点。
 *
 * 对于当前左臂链：
 *     base_link = WAIST_Y_S
 *     tip_link  = L_WRIST_R_S
 *
 *     结果相对于 WAIST_Y_S，不自动包含腰部相对于整机基座的运动；
 *     参考点为 L_WRIST_R_S 原点，不自动等于掌心或工具中心点 TCP。
 *     换用有偏移的 TCP 时，必须一致地调整正解和雅可比参考点。
 *
 * J(q) 随构型变化：
 *     机械臂姿态改变后，轴方向、轴原点或末端位置可能改变，
 *     所以通常需要根据当前 q 重新计算雅可比。
 *
 * 瞬时关系与有限运动：
 *     [v; omega] = J(q) * q_dot 是当前构型下的瞬时速度关系。
 *     不能把一次很大的关节位置变化直接乘当前 J，
 *     就当作末端有限位姿变化的精确结果。
 *     有限运动后的准确位姿仍应使用新的 q 做正运动学。
 *
 * 本函数只计算 J：
 *     不求逆，不求解关节速度，不执行逆运动学，不检查碰撞或限位。
 *     J 可以不是方阵，也可能秩不足。
 *     奇异构型并不意味着 J 本身不能计算；
 *     如何处理求解困难，是后续逆解或速度求解器的职责。
 */

namespace robot::motion
{

// 计算链末端原点的几何雅可比，不执行求逆或 IK。
// 几何雅可比表达 在当前构型下，各个关节的运动，会怎样影响末端的线速度和角速度？
//
// 上游：测试、速度映射，或后续数值逆解。
// model：初始化时构造的已验证urdf chain链；本函数不重新检查拓扑和下标。
// q：模型几何零位下的位置，旋转用 rad，移动用 m，不是增量。
//
// 返回 6 x N 矩阵 J：
//   [v_tip_in_base; omega_tip_in_base] = J(q) * q_dot

//   tip_link是选定的末端连杆
//   v_tip_in_base末端原点相对于链基准的线速度，用基准坐标轴表达
//   omega_tip_in_base末端连杆相对于链基准的角速度，用基准坐标轴表达

//   行 0~2：tip_link 原点相对于 base_link 的线速度部分。
//   行 3~5：tip_link 相对于 base_link 的角速度部分，不是 RPY 导数。
//   两部分都在 base_link 坐标轴下表达；列顺序与 q 一致。
//   q_dot 的旋转分量用 rad/s，移动分量用 m/s。
//   结果速度分别为 m/s 和 rad/s；base 的外部运动不计入。
//
// 固定关节参与坐标变换，但不增加矩阵列。
// 参考点是 tip_link 原点，不隐式附加掌心/TCP 偏移。
// 不检查限位或碰撞，不修改 model/q，不因奇异构型而拒绝计算。
//
// 实现：先调用 FK 求末端位置，再遍历链计算各列，共两遍链遍历。
// 这是解析几何公式，不是通过有限差分生成雅可比。
//
// 失败约定：
//   invalid_argument：q 非有限，由 forwardKinematics() 检查并抛出。
//   runtime_error：FK/中间变换/雅可比列出现无效数值。
//   logic_error：已验证模型出现不支持的类型，违反内部约定。
// 不返回部分结果；异常路径不作硬实时保证。
template <std::size_t N>
[[nodiscard]] Eigen::Matrix<double, 6, N> geometricJacobian(
    const ValidatedUrdfChain<N> &model,
    const std::array<double, N> &q)
{
    static_assert(N > 0, "Geometric Jacobian requires at least one active joint");
    using JacobianMatrix = Eigen::Matrix<double, 6, N>;

    // 第一遍：复用现有 FK，同时完成本次 q 的有限性检查。
    const Pose base_from_tip = forwardKinematics(model, q);
    const UrdfChain &chain = model.chain();

    JacobianMatrix jacobian = JacobianMatrix::Zero();
    Pose base_from_parent{};

    // 第二遍：获得每个关节的轴方向和轴上原点，都表达在 base 系中。
    for (const UrdfChainJoint &joint : chain.joints)
    {
        try
        {
            // 先应用安装变换，尚未叠加本关节的 q。
            // 上游关节的运动已经包含在 base_from_parent 中。
            Pose base_from_joint{};
            base_from_joint.position =
                base_from_parent.position +
                base_from_parent.orientation * joint.origin.position;
            base_from_joint.orientation =
                base_from_parent.orientation * joint.origin.orientation;
            base_from_joint = normalizedPose(base_from_joint);

            if (joint.type == UrdfJointType::Fixed)
            {
                // 固定关节没有雅可比列，但必须推进到它的子连杆。
                base_from_parent = base_from_joint;
                continue;
            }

            // 存在性、范围和顺序由 ValidatedUrdfChain 保证。
            const std::size_t q_index = *joint.q_index;
            const Eigen::Index column = static_cast<Eigen::Index>(q_index);

            // joint.axis 是关节局部轴；不能直接当作 base 系下的轴。
            const Eigen::Vector3d axis_in_base =
                base_from_joint.orientation * joint.axis;

            Eigen::Vector3d linear_part = Eigen::Vector3d::Zero();
            Eigen::Vector3d angular_part = Eigen::Vector3d::Zero();
            Pose base_from_child = base_from_joint;

            switch (joint.type)
            {
            case UrdfJointType::Revolute:
            case UrdfJointType::Continuous:
            {
                // Jv_i = a_i x (p_tip - p_joint), Jw_i = a_i。
                const Eigen::Vector3d joint_to_tip =
                    base_from_tip.position - base_from_joint.position;
                linear_part = axis_in_base.cross(joint_to_tip);
                angular_part = axis_in_base;

                // 推进到当前子连杆，供下一节使用。
                const Eigen::AngleAxisd angle_axis{q[q_index], joint.axis};
                const Eigen::Quaterniond local_rotation{angle_axis};
                base_from_child.orientation =
                    base_from_joint.orientation * local_rotation;
                break;
            }

            case UrdfJointType::Prismatic:
                // Jv_i = a_i, Jw_i = 0。
                linear_part = axis_in_base;
                base_from_child.position =
                    base_from_joint.position + axis_in_base * q[q_index];
                break;

            default:
                throw std::logic_error(
                    "Unsupported joint type in validated chain: " + joint.name);
            }

            // 有限位姿并不保证相减、叉乘后仍然有限。
            if (!linear_part.allFinite() || !angular_part.allFinite())
            {
                throw std::runtime_error(
                    "Non-finite Jacobian column at joint: " + joint.name);
            }

            jacobian.template block<3, 1>(0, column) = linear_part;
            jacobian.template block<3, 1>(3, column) = angular_part;

            // 检查动态计算结果，归一化累计姿态，再进入下一节。
            base_from_parent = normalizedPose(base_from_child);
        }
        catch (const std::invalid_argument &error)
        {
            throw std::runtime_error(
                "Invalid Jacobian state at joint " + joint.name +
                ": " + error.what());
        }
    }

    return jacobian;
}

} // namespace robot::motion