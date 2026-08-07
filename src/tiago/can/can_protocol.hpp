#pragma once

#include "can/can_frame.hpp"

#include <cstdint>

namespace robot::tiago
{
    enum class MotorControlCommand : std::uint8_t
    {
        Enable      = 0x01,
        Disable     = 0x02,
        ClearFault  = 0x03,
        Stop        = 0x12,
        QueryStatus = 0x20
    };

    struct MotorFeedback
    {
        std::uint8_t node_id{0};

        std::int32_t position_counts{0};
        std::int16_t velocity_counts_per_second{0};

        bool enabled{false};
        bool faulted{false};
        bool timed_out{false};

        std::uint8_t fault_code{0};
    };

    can::CanFrame encodeControlCommand(
        std::uint16_t node_id,
        MotorControlCommand command);

    can::CanFrame encodePositionCommand(
        std::uint16_t node_id,
        std::int32_t target_position_counts,
        std::uint16_t velocity_limit_counts_per_second);

    bool isFeedbackFrameId(
        std::uint16_t frame_id) noexcept;

    MotorFeedback decodeFeedbackFrame(
        const can::CanFrame &frame);
}