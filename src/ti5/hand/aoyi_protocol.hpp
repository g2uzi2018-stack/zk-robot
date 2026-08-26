#pragma once

#include "can/can_frame.hpp"

#include <chrono>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robot::ti5::hand
{

inline constexpr std::uint8_t kAoyiHeader1 = 0x55;
inline constexpr std::uint8_t kAoyiHeader2 = 0xAA;
inline constexpr std::uint8_t kAoyiMasterId = 0x01;
inline constexpr std::uint8_t kAoyiSetPositionsCommand = 0x50;
inline constexpr std::uint8_t kAoyiGetStatusCommand = 0x5F;
inline constexpr std::uint8_t kAoyiGetStatusSubcommand = 0x80;
inline constexpr std::size_t kAoyiMaxPayloadLength = 255;
inline constexpr std::size_t kAoyiChannelCount = 6;

using AoyiPositionValues =
    std::array<std::uint16_t, kAoyiChannelCount>;
using AoyiSpeedValues =
    std::array<std::uint8_t, kAoyiChannelCount>;
using AoyiForceValues =
    std::array<std::uint8_t, kAoyiChannelCount>;

struct AoyiHandStatus
{
    AoyiPositionValues positions{};
    std::optional<AoyiForceValues> forces;
};

struct AoyiPacket
{
    std::uint8_t hand_id{0};
    std::uint8_t master_id{kAoyiMasterId};
    std::uint8_t command{0};
    std::vector<std::uint8_t> payload;
};

std::uint8_t calculateLrc(const AoyiPacket &packet);

std::vector<std::uint8_t> encodePacket(
    std::uint8_t hand_id,
    std::uint8_t command,
    const std::vector<std::uint8_t> &payload);

std::optional<AoyiPacket> decodePacket(
    const std::vector<std::uint8_t> &bytes);

// 0x50 六通道位置命令。每路为 position_u16_le + speed_u8。
std::vector<std::uint8_t> encodePositionPayload(
    const AoyiPositionValues &positions,
    const AoyiSpeedValues &speeds);

// 解析当前已确认的状态字段：前 18 字节每三字节一路位置；payload
// 达到 24 字节时，最后六字节作为六路力值。未知中间字段保留不解释。
std::optional<AoyiHandStatus> decodeStatusPayload(
    const std::vector<std::uint8_t> &payload);

std::vector<robot::can::CanFrame> fragmentPacket(
    std::uint16_t can_id,
    const std::vector<std::uint8_t> &bytes);

class PacketReassembler final
{
public:
    PacketReassembler(
        std::uint16_t can_id,
        std::uint8_t hand_id,
        std::chrono::milliseconds inter_frame_timeout);

    std::optional<AoyiPacket> push(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point now =
            std::chrono::steady_clock::now());

    void reset() noexcept;

private:
    std::optional<AoyiPacket> tryExtract();

    std::uint16_t can_id_{0};
    std::uint8_t hand_id_{0};
    std::chrono::milliseconds inter_frame_timeout_{0};
    std::vector<std::uint8_t> bytes_;
    std::optional<std::chrono::steady_clock::time_point> last_frame_time_;
};

} // namespace robot::ti5::hand
