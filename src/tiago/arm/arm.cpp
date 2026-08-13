#include "tiago/arm/arm.hpp"

#include <stdexcept>
#include <unordered_set>

namespace robot::tiago
{
    Arm::Arm(const CanBusConfig &shoulder_config, const CanBusConfig &elbow_config, const CanBusConfig &wrist_config)
        : shoulder_bus_(shoulder_config.interface_name),
          elbow_bus_(elbow_config.interface_name),
          wrist_bus_(wrist_config.interface_name)
    {
        // 一条 7 自由度机械臂的固定结构：
        //
        // shoulder -> joint 1, joint 2
        // elbow    -> joint 3, joint 4
        // wrist    -> joint 5, joint 6, joint 7
        if (shoulder_config.joints.size() != 2)
        {
            throw std::invalid_argument("Arm shoulder config must contain exactly 2 joints");
        }

        if (elbow_config.joints.size() != 2)
        {
            throw std::invalid_argument("Arm elbow config must contain exactly 2 joints");
        }

        if (wrist_config.joints.size() != 3)
        {
            throw std::invalid_argument("Arm wrist config must contain exactly 3 joints");
        }

        // 一条 Arm 使用的三条 CAN 总线不能是同一个接口。
        if (shoulder_config.interface_name == elbow_config.interface_name ||
            shoulder_config.interface_name == wrist_config.interface_name ||
            elbow_config.interface_name == wrist_config.interface_name)
        {
            throw std::invalid_argument("Arm CAN interfaces must be different");
        }

        // 检查整条机械臂范围内的 Joint 名称不能重复。
        std::unordered_set<std::string> joint_names;

        const auto checkJointName = [&joint_names](const JointConfig &config)
        {
            if (!joint_names.insert(config.name).second)
            {
                throw std::invalid_argument("Duplicate joint name in Arm: " + config.name);
            }
        };

        for (const auto &config : shoulder_config.joints)
        {
            checkJointName(config);
        }

        for (const auto &config : elbow_config.joints)
        {
            checkJointName(config);
        }

        for (const auto &config : wrist_config.joints)
        {
            checkJointName(config);
        }

        // Arm 的 Joint 数量固定为 7，
        // 提前 reserve，避免构造过程中 vector 重新分配。
        joints_.reserve(kJointCount);

        // joint 1 ~ joint 2
        for (const auto &config : shoulder_config.joints)
        {
            joints_.emplace_back(config, shoulder_bus_);
        }

        // joint 3 ~ joint 4
        for (const auto &config : elbow_config.joints)
        {
            joints_.emplace_back(config, elbow_bus_);
        }

        // joint 5 ~ joint 7
        for (const auto &config : wrist_config.joints)
        {
            joints_.emplace_back(config, wrist_bus_);
        }
    }

    void Arm::enable()
    {
        for (auto &joint : joints_)
        {
            joint.enable();
        }
    }

    void Arm::disable()
    {
        for (auto &joint : joints_)
        {
            joint.disable();
        }
    }

    void Arm::clearFault()
    {
        for (auto &joint : joints_)
        {
            joint.clearFault();
        }
    }

    void Arm::stop()
    {
        for (auto &joint : joints_)
        {
            joint.stop();
        }
    }

    void Arm::commandPositions(const JointValues &positions, const JointValues &velocity_limits)
    {
        // 先检查完整的 7 Joint 命令。
        // 只要任意一个 Joint 参数非法，
        // 本次整臂命令就不会发送。
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            joints_[i].validateCommand(positions[i], velocity_limits[i]);
        }

        // 所有关节都检查通过以后，
        // 再真正发送 7 个位置命令。
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            joints_[i].commandPosition(positions[i], velocity_limits[i]);
        }
    }

    Arm::JointPositions Arm::readPositions()
    {
        JointPositions positions;

        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            positions[i] = joints_[i].readPosition();
        }

        return positions;
    }

    Joint &Arm::joint(std::size_t index)
    {
        return joints_.at(index);
    }

    const Joint &Arm::joint(std::size_t index) const
    {
        return joints_.at(index);
    }
}
