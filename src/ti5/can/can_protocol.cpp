#include "ti5/can/can_protocol.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace
{
    // Classic CAN 标准帧的 11-bit ID 范围。
    constexpr std::uint16_t kMaximumStandardCanId = 0x7FF;

    // Classic CAN 数据区的最大长度。
    constexpr std::uint8_t kMaximumCanDataLength = 8;

    // TI5 T170C 应用层命令码。
    constexpr std::uint8_t kPositionQueryCommand = 0x08;
    constexpr std::uint8_t kCspQueryCommand = 0x41;
    constexpr std::uint8_t kPositionCspCommand = 0x44;

    bool isStandardCanId(std::uint16_t can_id) noexcept
    {
        return can_id <= kMaximumStandardCanId;
    }

    // 校验节点 ID 是否可以作为标准 11-bit CAN ID 使用。
    void validateNodeId(std::uint16_t node_id)
    {
        if (!isStandardCanId(node_id))
        {
            throw std::invalid_argument("node_id must be a standard 11-bit CAN ID");
        }
    }

    // 按小端序将 int32 写入 CAN 数据区。
    void writeInt32LittleEndian(std::array<std::uint8_t, 8> &data,
                                std::size_t offset,
                                std::int32_t value)
    {
        const auto raw = static_cast<std::uint32_t>(value);
        data[offset] = static_cast<std::uint8_t>(raw & 0xFFU);
        data[offset + 1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
        data[offset + 2] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
        data[offset + 3] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
    }

    // 按小端序读取 int32，避免依赖有符号窄化转换的实现定义行为。
    std::int32_t readInt32LittleEndian(const std::array<std::uint8_t, 8> &data,
                                       std::size_t offset)
    {
        const auto raw = static_cast<std::uint32_t>(data[offset]) |
                         (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
                         (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
                         (static_cast<std::uint32_t>(data[offset + 3]) << 24U);

        if (raw <= static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max()))
        {
            return static_cast<std::int32_t>(raw);
        }

        const auto signed_value = static_cast<std::int64_t>(raw) - (std::int64_t{1} << 32U);
        return static_cast<std::int32_t>(signed_value);
    }

    // 按小端序读取 int16，避免依赖有符号窄化转换的实现定义行为。
    std::int16_t readInt16LittleEndian(const std::array<std::uint8_t, 8> &data,
                                       std::size_t offset)
    {
        const auto raw = static_cast<std::uint16_t>(data[offset]) |
                         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[offset + 1]) << 8U);

        if (raw <= static_cast<std::uint16_t>(std::numeric_limits<std::int16_t>::max()))
        {
            return static_cast<std::int16_t>(raw);
        }

        const auto signed_value = static_cast<std::int32_t>(raw) - (std::int32_t{1} << 16U);
        return static_cast<std::int16_t>(signed_value);
    }

    bool isPositionQueryResponseFormat(const robot::can::CanFrame &frame) noexcept
    {
        return isStandardCanId(frame.id) &&
               frame.data_length == 5 &&
               frame.data[0] == kPositionQueryCommand;
    }

    void validatePositionQueryResponse(const robot::can::CanFrame &frame)
    {
        if (!isPositionQueryResponseFormat(frame))
        {
            throw std::invalid_argument("Invalid TI5 0x08 position response frame");
        }
    }

    void validateCspFeedback(const robot::can::CanFrame &frame)
    {
        if (!isStandardCanId(frame.id))
        {
            throw std::invalid_argument("Invalid TI5 CSP feedback CAN ID");
        }

        if (frame.data_length != kMaximumCanDataLength)
        {
            throw std::invalid_argument("TI5 CSP feedback DLC must be 8");
        }
    }
}

namespace robot::ti5
{
    // 编码 0x08 读取当前位置请求帧。
    robot::can::CanFrame encodePositionQuery(std::uint16_t node_id)
    {
        validateNodeId(node_id);

        robot::can::CanFrame frame{};
        frame.id = node_id;
        frame.data_length = 1;
        frame.data[0] = kPositionQueryCommand;
        return frame;
    }

    // 判断帧是否为指定节点的 0x08 当前位置响应。
    bool isPositionQueryResponse(const robot::can::CanFrame &frame,
                                 std::uint16_t expected_node_id) noexcept
    {
        return isStandardCanId(expected_node_id) &&
               isPositionQueryResponseFormat(frame) &&
               frame.id == expected_node_id;
    }

    // 解码 0x08 响应中的 little-endian int32 位置计数。
    std::int32_t decodePositionCounts(const robot::can::CanFrame &frame)
    {
        validatePositionQueryResponse(frame);
        return readInt32LittleEndian(frame.data, 1);
    }

    // 编码 0x41 CSP 查询请求帧。
    robot::can::CanFrame encodeCspQuery(std::uint16_t node_id)
    {
        validateNodeId(node_id);

        robot::can::CanFrame frame{};
        frame.id = node_id;
        frame.data_length = 1;
        frame.data[0] = kCspQueryCommand;
        return frame;
    }

    // 判断帧是否为指定节点的 8 字节 CSP 反馈帧。
    bool isCspFeedback(const robot::can::CanFrame &frame,
                      std::uint16_t expected_node_id) noexcept
    {
        return isStandardCanId(expected_node_id) &&
               isStandardCanId(frame.id) &&
               frame.id == expected_node_id &&
               frame.data_length == kMaximumCanDataLength;
    }

    // 解码 8 字节 CSP 反馈帧。
    CspFeedback decodeCspFeedback(const robot::can::CanFrame &frame)
    {
        validateCspFeedback(frame);

        CspFeedback feedback;
        feedback.node_id = frame.id;
        feedback.current_milliamps = readInt16LittleEndian(frame.data, 0);
        feedback.speed_raw = readInt16LittleEndian(frame.data, 2);
        feedback.position_counts = readInt32LittleEndian(frame.data, 4);
        return feedback;
    }

    // 编码 0x44 Position CSP 目标位置帧。
    robot::can::CanFrame encodePositionCsp(std::uint16_t node_id,
                                           std::int32_t target_position_counts)
    {
        validateNodeId(node_id);

        robot::can::CanFrame frame{};
        frame.id = node_id;
        frame.data_length = 5;
        frame.data[0] = kPositionCspCommand;
        writeInt32LittleEndian(frame.data, 1, target_position_counts);
        return frame;
    }
}
