#include "tiago/head/head.hpp"

#include <stdexcept>

namespace robot::tiago
{
    Head::Head(const CanBusConfig &config)
        : bus_(config.interface_name)
    {
        // TIAGo Head 固定包含两个 Joint。
        if (config.joints.size() != kJointCount)
        {
            throw std::invalid_argument("Head config must contain exactly 2 joints");
        }

        // 两个 Joint 名称不能重复。
        if (config.joints[0].name == config.joints[1].name)
        {
            throw std::invalid_argument("Head joint names must be different");
        }

        // Head 两个关节必须都是旋转关节。
        for (const auto &joint_config : config.joints)
        {
            if (joint_config.motor.unit != JointUnit::Radian)
            {
                throw std::invalid_argument("Head joints must use radian units");
            }
        }

        joints_.reserve(kJointCount);

        // YAML 顺序：
        //
        // index 0 -> head_1_joint
        // index 1 -> head_2_joint
        for (const auto &joint_config : config.joints)
        {
            joints_.emplace_back(joint_config, bus_);
        }
    }

    void Head::enable()
    {
        for (auto &joint : joints_)
        {
            joint.enable();
        }
    }

    void Head::disable()
    {
        for (auto &joint : joints_)
        {
            joint.disable();
        }
    }

    void Head::clearFault()
    {
        for (auto &joint : joints_)
        {
            joint.clearFault();
        }
    }

    void Head::stop()
    {
        for (auto &joint : joints_)
        {
            joint.stop();
        }
    }

    void Head::commandPositions(const JointValues &positions, const JointValues &velocity_limits)
    {
        // 先验证完整的两轴命令。
        // 任意一个 Joint 非法，本次 Head 命令都不发送。
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            joints_[i].validateCommand(positions[i], velocity_limits[i]);
        }

        // 全部合法以后再统一发送。
        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            joints_[i].commandPosition(positions[i], velocity_limits[i]);
        }
    }

    Head::JointPositions Head::readPositions()
    {
        JointPositions positions;

        for (std::size_t i = 0; i < kJointCount; ++i)
        {
            positions[i] = joints_[i].readPosition();
        }

        return positions;
    }

    Joint &Head::joint(std::size_t index)
    {
        return joints_.at(index);
    }

    const Joint &Head::joint(std::size_t index) const
    {
        return joints_.at(index);
    }
}
