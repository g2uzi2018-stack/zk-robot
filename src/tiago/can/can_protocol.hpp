#pragma once

#include "can/can_frame.hpp"

#include <cstdint>

namespace robot::tiago
{
    // 电机控制命令码。
    enum class MotorControlCommand : std::uint8_t
    {
        Enable = 0x01,
        Disable = 0x02,
        ClearFault = 0x03,
        Stop = 0x12,
        QueryStatus = 0x20
    };

    // 电机反馈帧中解析出的运行状态。
    struct MotorFeedback
    {
        // CAN 节点 ID。
        std::uint8_t node_id{0};

        // 编码器位置和速度计数。
        std::int32_t position_counts{0};
        std::int16_t velocity_counts_per_second{0};

        // 电机状态标志。
        bool enabled{false};
        bool faulted{false};
        bool timed_out{false};

        // 电机故障码。
        std::uint8_t fault_code{0};
    };

    // 编码电机控制命令帧。
    can::CanFrame encodeControlCommand(std::uint16_t node_id, MotorControlCommand command);

    // 编码带目标位置和速度限制的位置控制命令帧。
    can::CanFrame encodePositionCommand(std::uint16_t node_id, std::int32_t target_position_counts, std::uint16_t velocity_limit_counts_per_second);

    // 编码速度控制命令帧。
    // target_velocity_counts_per_second 为有符号速度。
    can::CanFrame encodeVelocityCommand(std::uint16_t node_id, std::int32_t target_velocity_counts_per_second);

    // 判断帧 ID 是否属于电机反馈帧范围。
    bool isFeedbackFrameId(std::uint16_t frame_id) noexcept;

    // 解码电机反馈帧。
    MotorFeedback decodeFeedbackFrame(const can::CanFrame &frame);
}
