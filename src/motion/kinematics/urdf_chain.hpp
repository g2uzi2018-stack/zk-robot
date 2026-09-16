#pragma once

#include "motion/geometry/pose.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot::motion
{

    /** 当前支持的关节类型；固定关节不占用输入数组 q。 */
    enum class UrdfJointType
    {
        Fixed,
        Revolute,
        Continuous,
        Prismatic
    };

    /**
     * 一个关节位置的有限闭区间：[lower, upper]。
     *
     * 这是本工程的值类型，不是 Eigen 或 urdfdom 提供的类型。
     * lower、upper 都是标量，不是位置向量、Pose 或关节速度。
     * 它们与对应 q[i] 使用相同的模型零位、正方向和单位：
     *     旋转关节：rad；移动关节：m。
     *
     * 构造时验证：两个端点均有限，且 lower <= upper。
     * 不自动交换上下界，不把角度转换到 [-pi, pi]，不补默认上下界。
     * 允许 lower == upper，表示只允许一个位置；不会改变模型自由度。
     * 区间包含端点；安全余量应由上游明确收紧区间，不在这里隐式添加。
     *
     * 本类型只表示/检查一个标量范围，不执行 IK、不裁剪输入、不访问硬件。
     * 没有默认构造函数；缺失范围由外层 std::optional 显式表示。
     */
    class JointPositionBounds final
    {
    public:
        /**
         * 上游：URDF 加载实现、显式配置或测试代码。
         * 参数：准备保存的下界与上界，使用对应关节的模型坐标。
         * 成功时得到合法区间；失败时抛出 std::invalid_argument。
         * 不读取 XML，实际字段提取仍由 loadUrdfChain() 的实现负责。
         */
        JointPositionBounds(const double lower, const double upper)
            : lower_(lower), upper_(upper)
        {
            if (!std::isfinite(lower_) || !std::isfinite(upper_))
            {
                throw std::invalid_argument("Joint position bounds must be finite");
            }
            if (lower_ > upper_)
            {
                throw std::invalid_argument(
                    "Joint position lower bound must not exceed upper bound");
            }
        }

        /** 下游约束组装/诊断代码调用：按值返回下界，不修改对象。 */
        [[nodiscard]] double lower() const noexcept
        {
            return lower_;
        }

        /** 下游约束组装/诊断代码调用：按值返回上界，不修改对象。 */
        [[nodiscard]] double upper() const noexcept
        {
            return upper_;
        }

        /**
         * 上游：关节位置检查代码，传入一个 q[i]，不是整组 q。
         * 有限且位于闭区间内返回 true；越界、NaN 或无穷大返回 false。
         * 不修改 position，不夹紧到边界，也不因越界而抛异常。
         * 调用者决定如何报告错误；false 不能被当作一次成功的修正。
         */
        [[nodiscard]] bool contains(const double position) const noexcept
        {
            return std::isfinite(position) &&
                   position >= lower_ && position <= upper_;
        }

    private:
        /** 不暴露单独修改端点的接口；改变范围应重新构造合法区间。 */
        double lower_;
        double upper_;
    };

    /**
     * 运动链中的一个关节：保存模型数据，不保存当前关节位置。
     * 这是可编辑的原始数据；已验证几何模型会独立持有它的副本。
     */
    struct UrdfChainJoint
    {
        std::string name;
        std::string parent_link;
        std::string child_link;
        UrdfJointType type{UrdfJointType::Fixed};

        /**
         * URDF joint/origin：零位关节坐标系相对于父连杆的安装位姿。
         * position 使用 m，orientation 使用四元数；不是 visual/origin。
         */
        Pose origin{};

        /**
         * 关节局部坐标系中的运动轴；活动关节在模型初始化时归一化。
         * 旋转关节表示转轴，移动关节表示移动方向，固定关节不使用。
         * UnitX() 来自 Eigen，返回 (1, 0, 0)。
         */
        Eigen::Vector3d axis{Eigen::Vector3d::UnitX()};

        /** 活动关节对应 q 的下标；固定关节使用 nullopt，不占用 q。 */
        std::optional<std::size_t> q_index{std::nullopt};

        /**
         * 新增：模型中声明的关节位置上下界，对应 URDF limit/lower、upper。
         * 跟随本关节保存，因此与 name、type、q_index 位于同一个对象中。
         *
         * 必须结合 type 解释，不能仅凭 nullopt 就判定“不限位”：
         *
         * Revolute / Prismatic：
         *     有值：已保存一个数值合法的有限区间。
         *     nullopt：位置约束缺失/尚未加载；带约束 IK 入口必须拒绝，
         *              不能自动替换成无穷大或默认 [-pi, pi]。
         *
         * Continuous：
         *     本模型字段应为 nullopt，由关节类型明确表示无位置上下界。
         *     不表示真机绕线、控制器或任务层一定没有更严格的限制。
         *
         * Fixed：
         *     本字段应为 nullopt，没有独立关节位置需要约束。
         *
         * 范围数值有效，不等于整条链约束完整、类型匹配或真机安全。
         * 字段存在性与类型一致性，需要在带约束输入的初始化边界验证。
         * 当前 FK / Jacobian 只做几何计算，不因新增此字段而自动执行限位。
         *
         * 这是模型基线，不是合并控制器/任务限制后的最终有效约束。
         * 速度、加速度和 jerk 不属于本字段，也不是 IK 迭代步长。
         */
        std::optional<JointPositionBounds> position_bounds{std::nullopt};
    };

    /** 从指定基准连杆到指定末端连杆的原始运动链。 */
    struct UrdfChain
    {
        std::string base_link;
        std::string tip_link;

        /** 按 base -> tip 排列，包含固定关节。 */
        std::vector<UrdfChainJoint> joints;

        /** 只保存活动关节名称，顺序与 q 完全一致。 */
        std::vector<std::string> joint_names;
    };

    /**
     * 上游：初始化代码提供 URDF 文件路径、基准、末端和预期活动关节数。
     * 下游：返回原始 UrdfChain，供几何模型及后续约束初始化使用。
     *
     * 文件或运动链不合法时抛出 std::runtime_error。
     * 只在初始化时读文件，不访问硬件、不求解 IK、不执行限位。
     *
     * 这里只声明接口，XML 读取与字段填充由 urdf_chain.cpp 实现。
     * 在头文件增加 position_bounds，不会自动让旧加载实现填入限位。
     */
    UrdfChain loadUrdfChain(
        const std::string &urdf_path,
        const std::string &base_link,
        const std::string &tip_link,
        std::size_t expected_joint_count);

} // namespace robot::motion