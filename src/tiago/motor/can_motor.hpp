#pragma once

#include "can/socket_can.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/can/can_protocol.hpp"

#include <chrono>
#include <optional>

namespace robot::tiago
{
    // CAN 电机控制对象。
    //
    // 负责：
    //   - 通过 CAN 总线向指定节点发送电机命令
    //   - 根据编码器配置完成物理量与编码器计数之间的转换
    //   - 读取并解析当前电机反馈
    //
    // 不负责：
    //   - 关节名称
    //   - 机械位置限位
    //   - 关节最大速度限制
    //
    // 上述机器人关节层面的约束由后续 Joint 类负责。
    class CanMotor
    {
    public:
        // 使用单个电机配置和已经打开的 SocketCAN 总线创建对象。
        // SocketCan 的生命周期由外部负责管理，CanMotor 只保存对它的引用。
        CanMotor(const CanMotorConfig &config, robot::can::SocketCan &can);

        // 使能电机。
        void enable();

        // 禁用电机。
        void disable();

        // 清除电机故障状态。
        void clearFault();

        // 停止电机当前运动。
        void stop();

        // 查询一次当前电机状态；超时或未收到匹配反馈时返回 std::nullopt。
        std::optional<MotorFeedback> queryStatus();

        // 发送位置控制命令，参数单位由 CanMotorConfig::unit 决定。
        // CanMotor 只负责转换和发送，不检查机器人关节的机械限位。
        void commandPosition(double position, double velocity_limit);

        // 读取当前电机的位置反馈，返回值单位由 CanMotorConfig::unit 决定。
        std::optional<double> readPosition();

    private:
        // 等待电机反馈的默认超时时间。
        static constexpr std::chrono::milliseconds kFeedbackTimeout{10};

        // 从 CAN 总线上读取并校验当前电机节点的反馈。
        std::optional<MotorFeedback> readFeedback();

        // 当前电机的底层配置。
        CanMotorConfig config_;

        // 底层 CAN 总线，由外部负责创建和销毁。
        robot::can::SocketCan &can_;
    };
}
