#pragma once

#include "motion/geometry/pose.hpp"
#include "motion/kinematics/urdf_chain.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace robot::motion
{

// 已验证、独立持有且不可修改的单条 URDF 几何运动链。
// N 是活动关节数；固定关节保留在链内，但不占用 q。
// 本接口不支持 N == 0 的退化链。
//
// 初始化阶段：复制原始链，检查结构，归一化安装姿态与活动轴。
// 计算阶段：后续 FK / Jacobian 通过 chain() 读取，不重新验证拓扑。
//
// 坐标约定不变：
//   origin 是 joint/origin，表示零位关节坐标系相对于父连杆的位姿。
//   axis 始终表达在关节局部坐标系中，不转换到 base 坐标系。
//   q 是相对于模型几何零位的关节位置，旋转用 rad，移动用 m。
//   末端是 tip_link，参考系是 base_link，不隐式附加 TCP 变换。
//
// 本类型只验证已有的几何与索引数据，不验证关节限位或碰撞。
// 它不保存当前 q，也不读取文件、访问硬件或求解逆运动学。
template <std::size_t N>
class ValidatedUrdfChain final
{
    static_assert(N > 0, "ValidatedUrdfChain requires at least one active joint");

public:
    static constexpr std::size_t kJointCount = N;

    // 上游：初始化代码，传入 loadUrdfChain() 的结果或测试构造的链。
    // 验证失败时抛出 std::invalid_argument，不产生可使用的模型对象。
    // source 不会被修改；模型独立保存一份经过验证的数据。
    explicit ValidatedUrdfChain(const UrdfChain &source)
        : chain_(validateAndNormalize(source))
    {
    }

    // 下游：后续 FK / Jacobian，以及需要读取名称的初始化代码。
    // 不复制链、不分配内存、不执行验证。
    // 返回的引用仅在当前模型对象存活期间有效。
    [[nodiscard]] const UrdfChain &chain() const noexcept
    {
        return chain_;
    }

private:
    // 仅由构造函数调用。
    // 按值接收：从 source 复制一份工作数据，验证/归一化后返回给 chain_。
    static UrdfChain validateAndNormalize(UrdfChain chain)
    {
        if (chain.base_link.empty() || chain.tip_link.empty())
        {
            throw std::invalid_argument("Chain base/tip names must not be empty");
        }

        if (chain.base_link == chain.tip_link || chain.joints.empty())
        {
            throw std::invalid_argument(
                "Expected distinct base/tip links and a non-empty chain");
        }

        if (chain.joint_names.size() != N)
        {
            throw std::invalid_argument(
                "Active joint count mismatch: expected " + std::to_string(N) +
                ", got " + std::to_string(chain.joint_names.size()));
        }

        // 这些字符串和哈希集合仅在初始化阶段使用。
        std::string current_link = chain.base_link;
        std::unordered_set<std::string> visited_links{chain.base_link};
        std::unordered_set<std::string> visited_joints;
        std::size_t next_q_index = 0;

        for (auto &joint : chain.joints)
        {
            if (joint.name.empty())
            {
                throw std::invalid_argument("Joint name must not be empty");
            }

            if (!visited_joints.insert(joint.name).second)
            {
                throw std::invalid_argument("Repeated joint name: " + joint.name);
            }

            if (joint.parent_link != current_link || joint.child_link.empty())
            {
                throw std::invalid_argument("Broken chain at joint: " + joint.name);
            }

            if (!visited_links.insert(joint.child_link).second)
            {
                throw std::invalid_argument(
                    "Repeated link in chain: " + joint.child_link);
            }

            // 固定关节也必须具有合法的安装位姿。
            // normalizedPose() 是本工程 pose.hpp 中的函数，不是 Eigen API。
            try
            {
                joint.origin = normalizedPose(joint.origin);
            }
            catch (const std::invalid_argument &error)
            {
                throw std::invalid_argument(
                    "Invalid origin for joint " + joint.name + ": " + error.what());
            }

            switch (joint.type)
            {
            case UrdfJointType::Fixed:
                if (joint.q_index.has_value())
                {
                    throw std::invalid_argument(
                        "Fixed joint must not have q_index: " + joint.name);
                }
                // 固定关节不使用 axis，也不占用 q。
                break;

            case UrdfJointType::Revolute:
            case UrdfJointType::Continuous:
            case UrdfJointType::Prismatic:
            {
                if (!joint.q_index.has_value() || next_q_index >= N)
                {
                    throw std::invalid_argument(
                        "Missing or excess active joint index: " + joint.name);
                }

                if (*joint.q_index != next_q_index ||
                    chain.joint_names[next_q_index] != joint.name)
                {
                    throw std::invalid_argument(
                        "Joint order/index mismatch: " + joint.name);
                }

                if (!joint.axis.allFinite())
                {
                    throw std::invalid_argument(
                        "Joint axis must be finite: " + joint.name);
                }

                const double axis_norm = joint.axis.stableNorm();
                if (!std::isfinite(axis_norm) || axis_norm <= 1e-12)
                {
                    throw std::invalid_argument(
                        "Joint axis is zero, too small or unrepresentable: " +
                        joint.name);
                }

                // 只归一化长度，不改变方向或所在坐标系。
                joint.axis /= axis_norm;
                ++next_q_index;
                break;
            }

            default:
                throw std::invalid_argument(
                    "Unsupported joint type: " + joint.name);
            }

            current_link = joint.child_link;
        }

        if (current_link != chain.tip_link)
        {
            throw std::invalid_argument(
                "Chain does not end at the declared tip: " + chain.tip_link);
        }

        if (next_q_index != N)
        {
            throw std::invalid_argument(
                "Actual active joint count mismatch: expected " +
                std::to_string(N) + ", got " + std::to_string(next_q_index));
        }

        return chain;
    }

    // 保存实际对象而不是 source 的引用；外部修改 source 不会影响这里。
    // const 禁止构造完成后修改/赋值替换链；模型变更应重新构造。
    const UrdfChain chain_;
};

} // namespace robot::motion