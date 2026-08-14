#include "tiago/gripper/gripper.hpp"

#include <stdexcept>

namespace robot::tiago
{
    Gripper::Gripper(const CanBusConfig &config)
        : bus_(config.interface_name)
    {
        // 一只夹爪固定由两个独立 finger Joint 组成。
        if (config.joints.size() != kFingerCount)
        {
            throw std::invalid_argument("Gripper config must contain exactly 2 joints");
        }

        // 两个 finger 名称不能重复。
        if (config.joints[0].name == config.joints[1].name)
        {
            throw std::invalid_argument("Gripper finger joint names must be different");
        }

        // 当前 Gripper 只接受直线关节。
        for (const auto &joint_config : config.joints)
        {
            if (joint_config.motor.unit != JointUnit::Meter)
            {
                throw std::invalid_argument("Gripper finger joints must use meter units");
            }
        }

        fingers_.reserve(kFingerCount);

        // YAML 顺序约定：
        //
        // index 0 -> right finger
        // index 1 -> left finger
        for (const auto &joint_config : config.joints)
        {
            fingers_.emplace_back(joint_config, bus_);
        }
    }

    void Gripper::enable()
    {
        for (auto &finger : fingers_)
        {
            finger.enable();
        }
    }

    void Gripper::disable()
    {
        for (auto &finger : fingers_)
        {
            finger.disable();
        }
    }

    void Gripper::clearFault()
    {
        for (auto &finger : fingers_)
        {
            finger.clearFault();
        }
    }

    void Gripper::stop()
    {
        for (auto &finger : fingers_)
        {
            finger.stop();
        }
    }

    void Gripper::commandPositions(const FingerValues &positions, const FingerValues &velocity_limits)
    {
        // 先检查完整的两个 finger 命令。
        // 任意一个参数非法，本次夹爪命令都不会发送。
        for (std::size_t i = 0; i < kFingerCount; ++i)
        {
            fingers_[i].validateCommand(positions[i], velocity_limits[i]);
        }

        // 两个 finger 全部检查通过以后再发送。
        for (std::size_t i = 0; i < kFingerCount; ++i)
        {
            fingers_[i].commandPosition(positions[i], velocity_limits[i]);
        }
    }

    void Gripper::commandSymmetric(double finger_position, double velocity_limit)
    {
        const FingerValues positions{
            finger_position,
            finger_position};

        const FingerValues velocity_limits{
            velocity_limit,
            velocity_limit};

        commandPositions(positions, velocity_limits);
    }

    Gripper::FingerPositions Gripper::readPositions()
    {
        FingerPositions positions;

        for (std::size_t i = 0; i < kFingerCount; ++i)
        {
            positions[i] = fingers_[i].readPosition();
        }

        return positions;
    }

    Joint &Gripper::finger(std::size_t index)
    {
        return fingers_.at(index);
    }

    const Joint &Gripper::finger(std::size_t index) const
    {
        return fingers_.at(index);
    }
}
