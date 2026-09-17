#pragma once

#include "motion/kinematics/validated_urdf_chain.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motion::planning
{

    /**
     * 上游按关节名称提供最大加速度。
     *
     * Revolute / Continuous：
     *     rad/s^2
     *
     * Prismatic：
     *     m/s^2
     *
     * 这里保存的是关节侧限制，
     * 不是电机轴侧的 rpm/s 或编码器单位。
     */
    struct NamedJointAccelerationLimit
    {
        std::string joint_name;
        double max_acceleration{0.0};
    };

    /**
     * 把按名称提供的最大加速度整理成运动学 q 顺序。
     *
     * 输入：
     *
     *     L_WRIST_R    -> ...
     *     L_ELBOW_Y    -> ...
     *     L_SHOULDER_P -> ...
     *
     * 配置顺序可以任意。
     *
     * 输出：
     *
     *     q[0] -> max_acceleration[0]
     *     q[1] -> max_acceleration[1]
     *     ...
     *
     * 作用：
     *     后续加速度时间参数化只处理数组，
     *     不需要在计算循环里反复使用字符串找关节。
     *
     * 同时防止把错误关节的加速度限制套到另一个 q 上。
     */
    template <std::size_t N>
    class JointAccelerationLimits final
    {
    public:
        using AccelerationArray =
            std::array<double, N>;

        using NameArray =
            std::array<std::string, N>;

        explicit JointAccelerationLimits(
            const ValidatedUrdfChain<N> &model,
            const std::vector<NamedJointAccelerationLimit>
                &configured_limits)
        {
            std::unordered_map<std::string, double>
                by_name;

            /*
             * 第一步：
             * 检查上游配置本身。
             */
            for (const auto &configured :
                 configured_limits)
            {
                if (configured.joint_name.empty())
                {
                    throw std::invalid_argument(
                        "Acceleration-limit joint name "
                        "must not be empty");
                }

                if (!std::isfinite(
                        configured.max_acceleration) ||
                    configured.max_acceleration <= 0.0)
                {
                    throw std::invalid_argument(
                        "Joint max acceleration must "
                        "be finite and positive: " +
                        configured.joint_name);
                }

                if (!by_name
                         .emplace(
                             configured.joint_name,
                             configured.max_acceleration)
                         .second)
                {
                    throw std::invalid_argument(
                        "Duplicate joint acceleration limit: " +
                        configured.joint_name);
                }
            }

            std::array<bool, N> filled{};

            /*
             * 第二步：
             * 按 model 中真正的 q_index 排列。
             */
            for (const auto &joint :
                 model.chain().joints)
            {
                /*
                 * Fixed joint 没有 q，
                 * 所以不需要动态限制。
                 */
                if (!joint.q_index.has_value())
                {
                    continue;
                }

                const std::size_t q_index =
                    *joint.q_index;

                if (q_index >= N)
                {
                    throw std::logic_error(
                        "Joint q_index exceeds "
                        "acceleration-limit dimension");
                }

                const auto configured =
                    by_name.find(joint.name);

                if (configured == by_name.end())
                {
                    throw std::invalid_argument(
                        "Missing acceleration limit "
                        "for active joint: " +
                        joint.name);
                }

                joint_names_[q_index] =
                    joint.name;

                max_acceleration_[q_index] =
                    configured->second;

                filled[q_index] = true;

                /*
                 * 成功消费掉这一项。
                 *
                 * 最后如果 map 还有东西，
                 * 就说明配置里存在当前链没有的名字。
                 */
                by_name.erase(configured);
            }

            /*
             * 每一个 q 都必须有对应限制。
             */
            for (std::size_t i = 0;
                 i < N;
                 ++i)
            {
                if (!filled[i])
                {
                    throw std::logic_error(
                        "Incomplete q ordering while "
                        "building acceleration limits");
                }
            }

            /*
             * 不接受多余/拼错的关节名字。
             */
            if (!by_name.empty())
            {
                throw std::invalid_argument(
                    "Acceleration limit provided for "
                    "unknown active joint: " +
                    by_name.begin()->first);
            }
        }

        /**
         * 后续时间参数化调用。
         *
         * 返回：
         *
         *     q[0] 对应最大加速度
         *     q[1] 对应最大加速度
         *     ...
         */
        const AccelerationArray &
        maxAcceleration() const noexcept
        {
            return max_acceleration_;
        }

        /**
         * 主要用于诊断和报错。
         */
        const NameArray &
        jointNames() const noexcept
        {
            return joint_names_;
        }

        /**
         * 后续规划器入口调用。
         *
         * 防止这个限制对象来自：
         *
         *     另一条机械臂
         *     另一套 q 顺序
         *     另一份模型
         *
         * 名称和 q_index 顺序不一致就拒绝。
         */
        void requireJointOrder(
            const ValidatedUrdfChain<N> &model) const
        {
            std::size_t active_index = 0;

            for (const auto &joint :
                 model.chain().joints)
            {
                if (!joint.q_index.has_value())
                {
                    continue;
                }

                if (active_index >= N ||
                    *joint.q_index != active_index ||
                    joint_names_[active_index] !=
                        joint.name)
                {
                    throw std::invalid_argument(
                        "Joint acceleration limits "
                        "do not match model q order");
                }

                ++active_index;
            }

            if (active_index != N)
            {
                throw std::invalid_argument(
                    "Joint acceleration-limit dimension "
                    "does not match model");
            }
        }

    private:
        NameArray joint_names_{};

        AccelerationArray
            max_acceleration_{};
    };

} // namespace robot::motion::planning