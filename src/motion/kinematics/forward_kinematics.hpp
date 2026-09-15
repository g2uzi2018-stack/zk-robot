#pragma once

#include "motion/geometry/pose_composition.hpp"
#include "motion/kinematics/urdf_joint_transform.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace robot::motion
{

    // 上游：测试程序、规划程序或后续运动学模型。
    //
    // 输入：
    //   chain：已加载的运动链，joints 必须按 base -> tip 排列。
    //   q：模型坐标中的关节位置，顺序对应 chain.joint_names。
    //      旋转关节使用 rad，移动关节使用 m；固定关节不占用 q。
    //
    // 返回：
    //   chain.tip_link 相对于 chain.base_link 的当前位姿。
    //
    // 下游：显示、路径起点计算，或后续逆运动学的位姿误差计算。
    //
    // 不读取 URDF、不访问硬件、不检查限位和碰撞、不修改 q。
    // 本版每次检查链的连接和下标；尚未针对硬实时控制循环优化。
    template <std::size_t N>
    Pose forwardKinematics(const UrdfChain &chain, const std::array<double, N> &q)
    {
        static_assert(N > 0, "At least one active joint is required");

        if (chain.base_link.empty() || chain.tip_link.empty() ||
            chain.base_link == chain.tip_link || chain.joints.empty())
        {
            throw std::invalid_argument(
                "Invalid chain endpoints or empty chain");
        }

        if (chain.joint_names.size() != N)
        {
            throw std::invalid_argument(
                "Joint position count does not match chain");
        }

        for (const double position : q)
        {
            if (!std::isfinite(position))
            {
                throw std::invalid_argument(
                    "Joint positions must be finite");
            }
        }

        // 开始时，当前连杆就是基准自身：位置零、朝向单位旋转。
        Pose base_from_current{};
        std::string current_link = chain.base_link;

        // 下一个活动关节应该使用 q 的哪个元素。
        std::size_t next_q_index = 0;

        // base → 肩关节 → upper_arm → 肘关节 → forearm → 固定连接 → tool

        // 记录已经经过的连杆，防止链中出现重复连杆。
        std::unordered_set<std::string> visited_links{current_link};

        for (const auto &joint : chain.joints)
        {
            // 当前关节的父连杆，必须是前一轮已经算到的连杆。
            if (joint.name.empty() || joint.parent_link != current_link ||
                joint.child_link.empty())
            {
                throw std::invalid_argument("Broken chain at joint: " + joint.name);
            }

            // insert() 返回结果的 second 表示是否成功插入新元素。
            // 如果连杆已经出现过，就返回 false。
            if (!visited_links.insert(joint.child_link).second)
            {
                throw std::invalid_argument("Repeated link in chain: " + joint.child_link);
            }

            double joint_position = 0.0;

            if (joint.type == UrdfJointType::Fixed)
            {
                if (joint.q_index.has_value())
                {
                    throw std::invalid_argument("Fixed joint must not have q_index");
                }
            }
            else
            {
                if (!joint.q_index.has_value() || next_q_index >= N)
                {
                    throw std::invalid_argument("Missing or excess active joint index");
                }

                if (*joint.q_index != next_q_index || joint.name != chain.joint_names[next_q_index])
                {
                    throw std::invalid_argument("Joint order mismatch: " + joint.name);
                }

                joint_position = q.at(next_q_index);
                ++next_q_index;
            }

            // 算当前这一节，再接到累计结果上。
            const Pose parent_from_child = jointTransform(joint, joint_position);

            base_from_current = composePoses(base_from_current, parent_from_child);

            current_link = joint.child_link;
        }

        if (current_link != chain.tip_link || next_q_index != N)
        {
            throw std::invalid_argument(
                "Chain did not reach the expected tip/count");
        }

        return base_from_current;
    }

} // namespace robot::motion