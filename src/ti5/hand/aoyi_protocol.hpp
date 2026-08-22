#pragma once

#include "can/can_frame.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace robot::ti5::hand
{

inline constexpr std::uint8_t kAoyiHeader1 = 0x55;
inline constexpr std::uint8_t kAoyiHeader2 = 0xAA;
inline constexpr std::uint8_t kAoyiMasterId = 0x01;
inline constexpr std::uint8_t kAoyiGetStatusCommand = 0x5F;
inline constexpr std::uint8_t kAoyiGetStatusSubcommand = 0x80;
inline constexpr std::size_t kAoyiMaxPayloadLength = 255;

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
