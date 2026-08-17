#include "tiago/can/can_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
    // 控制命令帧和反馈帧的 CAN ID 基址。
    constexpr std::uint16_t kCommandIdBase = 0x100;
    constexpr std::uint16_t kFeedbackIdBase = 0x180;

    // CAN 节点 ID 的有效范围。
    constexpr std::uint16_t kMinimumNodeId = 1;
    constexpr std::uint16_t kMaximumNodeId = 127;

    // 经典 CAN 帧的数据长度。
    constexpr std::uint8_t kProtocolDataLength = 8;

    // 位置控制命令码。
    constexpr std::uint8_t kPositionCommandCode = 0x10;

    // 速度控制命令码。
    constexpr std::uint8_t kVelocityCommandCode = 0x11;

    // 反馈状态字段中的状态位掩码。
    constexpr std::uint8_t kEnabledMask = 0x01;
    constexpr std::uint8_t kFaultedMask = 0x02;
    constexpr std::uint8_t kTimedOutMask = 0x04;

    // 校验 CAN 节点 ID 是否处于协议允许的范围内。
    void validateNodeId(std::uint16_t node_id)
    {
        if (node_id < kMinimumNodeId || node_id > kMaximumNodeId)
        {
            throw std::invalid_argument("node_id must be in range 1..127");
        }
    }

    // 判断控制命令是否为协议支持的命令。
    bool isKnownControlCommand(robot::tiago::MotorControlCommand command) noexcept
    {
        using Command = robot::tiago::MotorControlCommand;

        switch (command)
        {
        case Command::Enable:
        case Command::Disable:
        case Command::ClearFault:
        case Command::Stop:
        case Command::QueryStatus:
            return true;

        default:
            return false;
        }
    }

    // 按小端序将 32 位有符号整数写入 CAN 数据区。
    void writeInt32LittleEndian(std::array<std::uint8_t, 8> &data, std::size_t offset, std::int32_t value)
    {
        const auto raw = static_cast<std::uint32_t>(value);
        data[offset] = static_cast<std::uint8_t>(raw & 0xFF);
        data[offset + 1] = static_cast<std::uint8_t>((raw >> 8) & 0xFF);
        data[offset + 2] = static_cast<std::uint8_t>((raw >> 16) & 0xFF);
        data[offset + 3] = static_cast<std::uint8_t>((raw >> 24) & 0xFF);
    }

    // 按小端序将 16 位无符号整数写入 CAN 数据区。
    void writeUint16LittleEndian(std::array<std::uint8_t, 8> &data, std::size_t offset, std::uint16_t value)
    {
        data[offset] = static_cast<std::uint8_t>(value & 0xFF);
        data[offset + 1] = static_cast<std::uint8_t>((value >> 8) & 0xFF);
    }

    // 按小端序从 CAN 数据区读取 32 位有符号整数。
    std::int32_t readInt32LittleEndian(const std::array<std::uint8_t, 8> &data, std::size_t offset)
    {
        const std::uint32_t raw = static_cast<std::uint32_t>(data[offset]) | (static_cast<std::uint32_t>(data[offset + 1]) << 8) | (static_cast<std::uint32_t>(data[offset + 2]) << 16) | (static_cast<std::uint32_t>(data[offset + 3]) << 24);
        if (raw <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
        {
            return static_cast<std::int32_t>(raw);
        }
        const std::int64_t signed_value = static_cast<std::int64_t>(raw) - (std::int64_t{1} << 32);
        return static_cast<std::int32_t>(signed_value);
    }

    // 按小端序从 CAN 数据区读取 16 位有符号整数。
    std::int16_t readInt16LittleEndian(const std::array<std::uint8_t, 8> &data, std::size_t offset)
    {
        const std::uint16_t raw = static_cast<std::uint16_t>(data[offset]) | (static_cast<std::uint16_t>(data[offset + 1]) << 8);
        if (raw <= static_cast<std::uint16_t>(std::numeric_limits<std::int16_t>::max()))
        {
            return static_cast<std::int16_t>(raw);
        }
        const std::int32_t signed_value = static_cast<std::int32_t>(raw) - (std::int32_t{1} << 16);
        return static_cast<std::int16_t>(signed_value);
    }
}

namespace robot::tiago
{
    // 编码电机控制命令帧。
    can::CanFrame encodeControlCommand(std::uint16_t node_id, MotorControlCommand command)
    {
        validateNodeId(node_id);

        if (!isKnownControlCommand(command))
        {
            throw std::invalid_argument("Unknown motor control command");
        }

        can::CanFrame frame;
        frame.id = static_cast<std::uint16_t>(kCommandIdBase + node_id);
        frame.data_length = kProtocolDataLength;
        frame.data[6] = static_cast<std::uint8_t>(command);
        return frame;
    }

    // 编码带目标位置和速度限制的位置控制命令帧。
    can::CanFrame encodePositionCommand(std::uint16_t node_id, std::int32_t target_position_counts, std::uint16_t velocity_limit_counts_per_second)
    {
        validateNodeId(node_id);

        can::CanFrame frame;
        frame.id = static_cast<std::uint16_t>(kCommandIdBase + node_id);
        frame.data_length = kProtocolDataLength;
        writeInt32LittleEndian(frame.data, 0, target_position_counts);
        writeUint16LittleEndian(frame.data, 4, velocity_limit_counts_per_second);
        frame.data[6] = kPositionCommandCode;
        return frame;
    }

    // 判断帧 ID 是否属于电机反馈帧范围。
    bool isFeedbackFrameId(std::uint16_t frame_id) noexcept
    {
        return frame_id > kFeedbackIdBase && frame_id <= kFeedbackIdBase + kMaximumNodeId;
    }

    // 解码电机反馈帧并提取状态、位置和速度信息。
    MotorFeedback decodeFeedbackFrame(const can::CanFrame &frame)
    {
        if (!isFeedbackFrameId(frame.id))
        {
            throw std::invalid_argument("Invalid feedback frame ID: " + std::to_string(frame.id));
        }

        if (frame.data_length != kProtocolDataLength)
        {
            throw std::invalid_argument("Feedback frame length must be 8");
        }

        MotorFeedback feedback;
        feedback.node_id = static_cast<std::uint8_t>(frame.id - kFeedbackIdBase);
        feedback.position_counts = readInt32LittleEndian(frame.data, 0);
        feedback.velocity_counts_per_second = readInt16LittleEndian(frame.data, 4);
        const std::uint8_t status = frame.data[6];
        feedback.enabled = (status & kEnabledMask) != 0;
        feedback.faulted = (status & kFaultedMask) != 0;
        feedback.timed_out = (status & kTimedOutMask) != 0;
        feedback.fault_code = frame.data[7];
        return feedback;
    }

    // 编码速度控制命令帧。
    can::CanFrame encodeVelocityCommand(std::uint16_t node_id, std::int32_t target_velocity_counts_per_second)
    {
        validateNodeId(node_id);

        can::CanFrame frame;
        frame.id = static_cast<std::uint16_t>(kCommandIdBase + node_id);
        frame.data_length = kProtocolDataLength;
        writeInt32LittleEndian(frame.data, 0, target_velocity_counts_per_second);

        // byte 4..5 对 velocity command 不使用，
        // 保持默认 0。
        frame.data[6] = kVelocityCommandCode;

        return frame;
    }
}
