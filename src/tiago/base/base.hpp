#pragma once

#include "tiago/base/base_config.hpp"
#include "tiago/can/can_bus.hpp"
#include "tiago/motor/can_motor.hpp"

namespace robot::tiago
{
    // 移动底盘。对外提供整车线速度和角速度接口，隐藏左右轮的控制细节。
    class Base
    {
    public:
        // 使用底盘配置创建对象，并建立左右轮共享的 CAN 总线。
        explicit Base(const BaseConfig &config);

        // 使能左右轮电机。
        void enable();

        // 禁用左右轮电机。
        void disable();

        // 清除左右轮电机故障。
        void clearFault();

        // 停止左右轮运动。
        void stop();

        // linear_velocity 单位为 m/s；angular_velocity 单位为 rad/s。
        // 约定 angular_velocity > 0 时底盘向左转。
        void commandVelocity(double linear_velocity, double angular_velocity);

        // 只校验整车速度命令，不发送 CAN 数据。
        void validateVelocityCommand(double linear_velocity, double angular_velocity) const;

    private:
        CanBus bus_;
        CanMotor right_motor_;
        CanMotor left_motor_;

        double wheel_radius_;
        double wheel_separation_;
        double max_linear_velocity_;
        double max_angular_velocity_;
    };
}
