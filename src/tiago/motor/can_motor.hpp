#pragma once

#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/can/can_protocol.hpp"

#include <chrono>
#include <optional>

namespace robot::tiago
{
    // CAN 电机控制对象。
    //
    // CanMotor 负责：
    //   - 当前电机节点的控制命令
    //   - 编码器物理量转换
    //   - 获取当前电机状态
    //
    // CanMotor 不直接管理 SocketCAN。
    //
    // 同一条 CAN 总线上的多个 CanMotor
    // 共享一个 CanBus。
    class CanMotor
    {
    public:
        // 使用电机配置和所属 CAN 总线创建电机对象。
        //
        // CanBus 的生命周期由外部负责管理。
        CanMotor(const CanMotorConfig &config, CanBus &bus);

        // 使能电机。
        void enable();

        // 禁用电机。
        void disable();

        // 清除电机故障。
        void clearFault();

        // 停止当前电机运动。
        void stop();

        // 主动查询当前电机状态。
        std::optional<MotorFeedback> queryStatus();

        // 发送位置控制命令。
        //
        // position 和 velocity_limit 的单位
        // 由 CanMotorConfig::unit 决定。
        //
        // CanMotor 不检查机器人关节机械限位。
        void commandPosition(double position, double velocity_limit);

        // 主动查询并返回当前电机位置。
        //
        // Radian:
        //   返回 rad
        //
        // Meter:
        //   返回 m
        std::optional<double> readPosition();

    private:
        // 等待电机反馈的默认超时时间。
        static constexpr std::chrono::milliseconds kFeedbackTimeout{10};

        // 当前电机配置。
        CanMotorConfig config_;

        // 当前电机所属的共享 CAN 总线。
        CanBus &bus_;
    };
}
