#pragma once

#include "motion/kinematics/validated_urdf_chain.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motion
{

    /**
     * 一条已经解析好的外部位置约束，不负责读取 YAML/JSON。
     * joint_name 必须是当前链中的活动关节名，不是电机 ID 或连杆名。
     * bounds 必须已换算到该关节的模型零位、正方向和单位：rad 或 m。
     * 上游可以从实际关节配置、任务配置或测试常量构造它。
     * 没有提供某关节的配置，就不添加该项；不能用 [0, 0] 代表“未提供”。
     */
    struct NamedJointPositionBounds
    {
        std::string joint_name;
        JointPositionBounds bounds;
    };


    //  | 类型                            | 回答的问题                             |
//      | ----------------------------- | --------------------------------- |
//      | `JointPositionBounds`         | **这一个关节**允许什么位置？                  |
//      | `JointPositionConstraints<N>` | **这整条链**按 `q` 顺序排列后，每个关节最终允许什么位置？ |

    /**
     * 一条链按 q 顺序排列的有效位置约束。
     *
     * 初始化时：按关节名匹配配置、合并模型范围、检查完整性并保存副本。
     * 计算时：只读数组，或调用 contains(q)；不再查名字、解析文件或合并。
     *
     * 合并规则：
     *   模型和配置都有范围：取交集；空交集报错，不允许配置放宽模型范围。
     *   有界关节模型缺范围：可由明确配置补齐；两边都缺失则构造失败。
     *   Continuous：模型不应保存位置上下界；允许外部配置施加有限范围。
     *   Fixed：没有 q 元素，不生成约束项，也不接受外部位置范围。
     *
     * 成功构造后，bounds()[i] 对应 q[i]：
     *   有值：有限闭区间，包含两个端点。
     *   nullopt：仅表示未附加有限位置范围的 Continuous，不再表示数据缺失。
     * 即使没有位置上下界，q[i] 也必须是有限数。
     * 连续关节使用未包裹的位置值，不在这里对角度取模或自动跨越 2*pi。
     *
     * 本对象只验证单关节位置范围，不验证碰撞、速度、加速度或目标可达性。
     * 上游必须提供完整的实际执行约束；本类不会自动从控制器读取这些数据。
     * 配置每个关节最多一项；多份外部来源应先由上游明确整理，不能静默覆盖。
     *
     * 使用约定：与构造时的模型配套使用。模型或标定改变后重新构造。
     * requireJointOrder() 只检查名称/类型/顺序，不能证明零位与方向标定正确。
     * 参数/约束不合法时抛出 invalid_argument；不返回部分约束，不修改模型和配置。
     */
    template <std::size_t N>
    class JointPositionConstraints final
    {
        static_assert(N > 0, "JointPositionConstraints requires active joints");

    public:
        using JointVector = std::array<double, N>;
        using BoundsArray = std::array<std::optional<JointPositionBounds>, N>;
        using NameArray = std::array<std::string, N>;

        /** 初始化代码调用：model 提供已验证几何和 q 顺序，configured 提供外部范围。 */
        explicit JointPositionConstraints(const ValidatedUrdfChain<N> &model, const std::vector<NamedJointPositionBounds> &configured = {}) : data_(build(model, configured))
        {
        }

        /** 供后续求解器读取上下界；返回只读引用，不复制数组。 */
        [[nodiscard]] const BoundsArray &bounds() const noexcept
        {
            return data_.bounds;
        }

        /** 供初始化核对或诊断打印：第 i 个名称对应 q[i] 和 bounds()[i]。 */
        [[nodiscard]] const NameArray &jointNames() const noexcept
        {
            return data_.names;
        }

        /**
         * 后续求解器/规划器调用：检查整组候选 q 是否有限且满足位置范围。
         * true 只代表位置范围检查通过，不代表 IK 到达目标或轨迹安全。
         * false 不修改、不裁剪 q，也不自动搜索另一个解；不分配内存、不抛异常。
         */
        [[nodiscard]] bool contains(const JointVector &q) const noexcept
        {
            for (std::size_t i = 0; i < N; ++i)
            {
                if (!std::isfinite(q[i]) ||
                    (data_.bounds[i] && !data_.bounds[i]->contains(q[i])))
                {
                    return false;
                }
            }
            return true;
        }

        /**
         * 后续求解器初始化时调用，避免同为 N 自由度却把关节顺序接错。
         * 不需要在每轮迭代调用；失败抛出 invalid_argument。
         * 这不是完整模型身份/标定检查；同名同类型但几何或零位不同仍需重新组装。
         */
        void requireJointOrder(const ValidatedUrdfChain<N> &model) const
        {
            for (const auto &joint : model.chain().joints)
            {
                if (joint.type == UrdfJointType::Fixed)
                {
                    continue;
                }
                const std::size_t i = *joint.q_index;
                if (data_.names[i] != joint.name || data_.types[i] != joint.type)
                {
                    throw std::invalid_argument(
                        "Constraint joint order/type mismatch at q[" + std::to_string(i) + "]");
                }
            }
        }

    private:
        /** 只是将本对象需要保存的三个数组放在一起，不是新的模型或求解器。 */
        struct Data
        {
            BoundsArray bounds{};
            NameArray names{};
            std::array<UrdfJointType, N> types{};
        };

        /** 仅构造函数调用：先复制模型范围，再按名字合并配置，最后检查完整性。 */
        static Data build(const ValidatedUrdfChain<N> &model, const std::vector<NamedJointPositionBounds> &configured)
        {
            Data result;
            std::unordered_map<std::string, std::size_t> index_by_name;

            for (const auto &joint : model.chain().joints)
            {
                switch (joint.type)
                {
                case UrdfJointType::Fixed:
                    if (joint.position_bounds)
                    {
                        throw std::invalid_argument(
                            "Fixed joint must not have model position bounds: " + joint.name);
                    }
                    continue;

                case UrdfJointType::Continuous:
                    if (joint.position_bounds)
                    {
                        throw std::invalid_argument(
                            "Use external bounds for continuous joint: " + joint.name);
                    }
                    break;

                case UrdfJointType::Revolute:
                case UrdfJointType::Prismatic:
                    break; // 缺失的模型范围暂时保留，允许配置随后补齐。

                default:
                    throw std::invalid_argument("Unsupported joint type: " + joint.name);
                }

                // q_index 的存在性/顺序/范围由已验证几何模型保证。
                const std::size_t i = *joint.q_index;
                result.names[i] = joint.name;
                result.types[i] = joint.type;
                result.bounds[i] = joint.position_bounds;
                index_by_name.emplace(joint.name, i);
            }

            std::array<bool, N> configured_seen{};
            for (const auto &entry : configured)
            {
                const auto found = index_by_name.find(entry.joint_name);
                if (found == index_by_name.end())
                {
                    throw std::invalid_argument(
                        "Configured name is not an active joint in this chain: " + entry.joint_name);
                }
                const std::size_t i = found->second;
                if (configured_seen[i])
                {
                    throw std::invalid_argument("Duplicate configured joint: " + entry.joint_name);
                }
                configured_seen[i] = true;

                auto &effective = result.bounds[i];
                if (!effective)
                {
                    effective = entry.bounds;
                    continue;
                }

                const double lower = std::max(effective->lower(), entry.bounds.lower());
                const double upper = std::min(effective->upper(), entry.bounds.upper());
                if (lower > upper)
                {
                    throw std::invalid_argument(
                        "Empty position-bound intersection for joint: " + entry.joint_name);
                }
                effective = JointPositionBounds{lower, upper};
            }

            for (std::size_t i = 0; i < N; ++i)
            {
                if (result.types[i] != UrdfJointType::Continuous && !result.bounds[i])
                {
                    throw std::invalid_argument(
                        "No model or configured position bounds for joint: " + result.names[i]);
                }
            }
            return result;
        }

        /** 保存独立只读副本；修改外部配置不影响它，新要求应重新构造。 */
        const Data data_;
    };

} // namespace robot::motion