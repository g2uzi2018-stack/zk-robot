#include "tiago/joint/joint.hpp"

#include <stdexcept>

namespace robot::tiago
{
    Joint::Joint(const JointConfig &config, CanBus &bus)
        : name_(config.name),
          limits_(config.limits),
          motor_(config.motor, bus)
    {
    }

    // 返回关节名称。
    const std::string &Joint::name() const noexcept
    {
        return name_;
    }

    // 使能当前关节对应的电机。
    void Joint::enable()
    {
        motor_.enable();
    }

    // 禁用当前关节对应的电机。
    void Joint::disable()
    {
        motor_.disable();
    }

    // 清除当前关节对应电机的故障。
    void Joint::clearFault()
    {
        motor_.clearFault();
    }

    // 停止当前关节运动。
    void Joint::stop()
    {
        motor_.stop();
    }

    // 检查目标位置是否处于关节机械允许范围内。
    void Joint::validateTargetPosition(double position) const
    {
        if (position < limits_.min_position ||
            position > limits_.max_position)
        {
            throw std::out_of_range("Target position exceeds joint limits");
        }
    }

    // 检查本次运动速度是否超过关节允许范围。
    void Joint::validateVelocityLimit(double velocity_limit) const
    {
        if (velocity_limit < 0.0)
        {
            throw std::out_of_range("Velocity limit must not be negative");
        }

        if (velocity_limit > limits_.max_velocity)
        {
            throw std::out_of_range("Velocity limit exceeds joint max_velocity");
        }
    }

    // 控制关节运动到目标位置。
    //
    // Joint 只负责机器人层面的约束检查。
    // 检查通过后，将实际控制交给 CanMotor。
    void Joint::commandPosition(double position, double velocity_limit)
    {
        validateTargetPosition(position);
        validateVelocityLimit(velocity_limit);

        motor_.commandPosition(position, velocity_limit);
    }

    // 读取当前关节位置。
    //
    // 实际 CAN 查询和编码器转换由 CanMotor 完成。
    std::optional<double> Joint::readPosition()
    {
        return motor_.readPosition();
    }
}
