#pragma once

#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/motor/can_motor.hpp"

#include <optional>
#include <string>

namespace robot::tiago
{
    // 机器人关节对象。
    //
    // Joint 位于机器人语义层，负责：
    //   - 保存关节名称
    //   - 检查机械位置限制
    //   - 检查最大速度限制
    //   - 调用对应的 CanMotor 执行实际电机控制
    //
    // Joint 不负责：
    //   - CAN 协议编码
    //   - SocketCAN 收发
    //   - 编码器 counts 与物理量之间的转换
    //
    // 这些底层工作由 CanMotor 和 CanBus 负责。
    class Joint
    {
    public:
        // 根据关节配置和所属 CAN 总线创建 Joint。
        //
        // 一个 Joint 当前对应一个 CanMotor。
        // CanBus 生命周期由外部负责管理。
        Joint(const JointConfig &config, CanBus &bus);

        // 获取关节名称。
        const std::string &name() const noexcept;

        // 使能当前关节对应的电机。
        void enable();

        // 禁用当前关节对应的电机。
        void disable();

        // 清除当前关节对应电机的故障。
        void clearFault();

        // 停止当前关节运动。
        void stop();

        // 控制关节运动到目标位置。
        //
        // position:
        //   目标关节位置。
        //
        // velocity_limit:
        //   本次运动允许的最大速度。
        //
        // Joint 会检查：
        //   min_position <= position <= max_position
        //   0 <= velocity_limit <= max_velocity
        //
        // 检查通过后交给 CanMotor 执行。
        void commandPosition(double position, double velocity_limit);

        // 读取当前关节位置。
        //
        // 返回单位由该关节的 motor 配置决定：
        //   Radian -> rad
        //   Meter  -> m
        std::optional<double> readPosition();

    private:
        // 检查目标位置是否处于机械允许范围。
        void validateTargetPosition(double position) const;

        // 检查速度是否处于关节允许范围。
        void validateVelocityLimit(double velocity_limit) const;

        // 关节名称。
        std::string name_;

        // 机械位置和速度限制。
        JointLimits limits_;

        // 当前关节对应的底层电机。
        CanMotor motor_;
    };
}
