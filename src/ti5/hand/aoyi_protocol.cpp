#include "ti5/hand/aoyi_protocol.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>

namespace robot::ti5::hand
{

std::uint8_t calculateLrc(const AoyiPacket &packet)
{
    std::uint8_t lrc = 0;
    lrc = static_cast<std::uint8_t>(lrc ^ packet.hand_id);
    lrc = static_cast<std::uint8_t>(lrc ^ packet.master_id);
    lrc = static_cast<std::uint8_t>(lrc ^ packet.command);
    lrc = static_cast<std::uint8_t>(
        lrc ^ static_cast<std::uint8_t>(packet.payload.size()));
    for (const auto byte : packet.payload)
    {
        lrc = static_cast<std::uint8_t>(lrc ^ byte);
    }
    return lrc;
}

std::vector<std::uint8_t> encodePacket(
    const std::uint8_t hand_id,
    const std::uint8_t command,
    const std::vector<std::uint8_t> &payload)
{
    if (hand_id == 0)
    {
        throw std::invalid_argument("Aoyi hand ID must be non-zero");
    }
    if (payload.size() > kAoyiMaxPayloadLength)
    {
        throw std::invalid_argument("Aoyi payload exceeds one-byte length field");
    }

    AoyiPacket packet;
    packet.hand_id = hand_id;
    packet.master_id = kAoyiMasterId;
    packet.command = command;
    packet.payload = payload;

    std::vector<std::uint8_t> bytes{
        kAoyiHeader1,
        kAoyiHeader2,
        packet.hand_id,
        packet.master_id,
        packet.command,
        static_cast<std::uint8_t>(packet.payload.size())};
    bytes.insert(bytes.end(), packet.payload.begin(), packet.payload.end());
    bytes.push_back(calculateLrc(packet));
    return bytes;
}

std::optional<AoyiPacket> decodePacket(
    const std::vector<std::uint8_t> &bytes)
{
    if (bytes.size() < 7 ||
        bytes[0] != kAoyiHeader1 ||
        bytes[1] != kAoyiHeader2)
    {
        return std::nullopt;
    }

    const auto payload_length = static_cast<std::size_t>(bytes[5]);
    if (bytes.size() != 7 + payload_length ||
        bytes[2] == 0 ||
        bytes[3] != kAoyiMasterId)
    {
        return std::nullopt;
    }

    AoyiPacket packet;
    packet.hand_id = bytes[2];
    packet.master_id = bytes[3];
    packet.command = bytes[4];
    packet.payload.assign(
        bytes.begin() + 6,
        bytes.begin() + 6 + static_cast<std::ptrdiff_t>(payload_length));

    if (calculateLrc(packet) != bytes.back())
    {
        return std::nullopt;
    }
    return packet;
}

std::vector<std::uint8_t> encodePositionPayload(
    const AoyiPositionValues &positions,
    const AoyiSpeedValues &speeds)
{
    std::vector<std::uint8_t> payload;
    payload.reserve(kAoyiChannelCount * 3);
    for (std::size_t index = 0; index < kAoyiChannelCount; ++index)
    {
        payload.push_back(static_cast<std::uint8_t>(
            positions[index] & 0xFFU));
        payload.push_back(static_cast<std::uint8_t>(
            (positions[index] >> 8U) & 0xFFU));
        payload.push_back(speeds[index]);
    }
    return payload;
}

std::optional<AoyiHandStatus> decodeStatusPayload(
    const std::vector<std::uint8_t> &payload)
{
    constexpr std::size_t position_bytes = kAoyiChannelCount * 3;
    if (payload.size() < position_bytes)
    {
        return std::nullopt;
    }

    AoyiHandStatus status;
    for (std::size_t index = 0; index < kAoyiChannelCount; ++index)
    {
        const auto offset = index * 3;
        status.positions[index] = static_cast<std::uint16_t>(
            static_cast<std::uint16_t>(payload[offset]) |
            (static_cast<std::uint16_t>(payload[offset + 1]) << 8U));
    }

    if (payload.size() >= position_bytes + kAoyiChannelCount)
    {
        AoyiForceValues forces{};
        const auto force_offset = payload.size() - kAoyiChannelCount;
        std::copy_n(
            payload.begin() + static_cast<std::ptrdiff_t>(force_offset),
            kAoyiChannelCount,
            forces.begin());
        status.forces = forces;
    }
    return status;
}

std::vector<robot::can::CanFrame> fragmentPacket(
    const std::uint16_t can_id,
    const std::vector<std::uint8_t> &bytes)
{
    if (can_id == 0 || can_id > 0x7FF)
    {
        throw std::invalid_argument("Aoyi CAN ID must be in 1..2047");
    }
    if (bytes.empty())
    {
        throw std::invalid_argument("Cannot fragment an empty Aoyi packet");
    }

    std::vector<robot::can::CanFrame> frames;
    for (std::size_t offset = 0; offset < bytes.size();)
    {
        const auto length = std::min<std::size_t>(8, bytes.size() - offset);
        robot::can::CanFrame frame;
        frame.id = can_id;
        frame.data_length = static_cast<std::uint8_t>(length);
        std::copy_n(
            bytes.begin() + static_cast<std::ptrdiff_t>(offset),
            length,
            frame.data.begin());
        frames.push_back(frame);
        offset += length;
    }
    return frames;
}

PacketReassembler::PacketReassembler(
    const std::uint16_t can_id,
    const std::uint8_t hand_id,
    const std::chrono::milliseconds inter_frame_timeout)
    : can_id_(can_id),
      hand_id_(hand_id),
      inter_frame_timeout_(inter_frame_timeout)
{
    if (can_id_ == 0 || can_id_ > 0x7FF)
    {
        throw std::invalid_argument("Aoyi CAN ID must be in 1..2047");
    }
    if (hand_id_ == 0)
    {
        throw std::invalid_argument("Aoyi hand ID must be non-zero");
    }
    if (inter_frame_timeout_.count() <= 0)
    {
        throw std::invalid_argument(
            "Aoyi inter-frame timeout must be positive");
    }
}

std::optional<AoyiPacket> PacketReassembler::push(
    const robot::can::CanFrame &frame,
    const std::chrono::steady_clock::time_point now)
{
    if (frame.id != can_id_)
    {
        return std::nullopt;
    }
    if (last_frame_time_ &&
        (now < *last_frame_time_ ||
         now - *last_frame_time_ > inter_frame_timeout_))
    {
        reset();
    }
    if (frame.data_length > 8)
    {
        reset();
        return std::nullopt;
    }
    if (frame.data_length == 0)
    {
        return std::nullopt;
    }

    bytes_.insert(
        bytes_.end(),
        frame.data.begin(),
        frame.data.begin() + frame.data_length);
    last_frame_time_ = now;
    return tryExtract();
}

void PacketReassembler::reset() noexcept
{
    bytes_.clear();
    last_frame_time_.reset();
}

std::optional<AoyiPacket> PacketReassembler::tryExtract()
{
    const std::array<std::uint8_t, 2> header{
        kAoyiHeader1,
        kAoyiHeader2};

    while (true)
    {
        if (bytes_.size() < header.size())
        {
            return std::nullopt;
        }

        const auto header_position = std::search(
            bytes_.begin(),
            bytes_.end(),
            header.begin(),
            header.end());
        if (header_position == bytes_.end())
        {
            if (!bytes_.empty() && bytes_.back() == kAoyiHeader1)
            {
                const auto last = bytes_.back();
                bytes_.clear();
                bytes_.push_back(last);
            }
            else
            {
                bytes_.clear();
            }
            return std::nullopt;
        }
        if (header_position != bytes_.begin())
        {
            bytes_.erase(bytes_.begin(), header_position);
        }
        if (bytes_.size() < 6)
        {
            return std::nullopt;
        }

        if (bytes_[2] != hand_id_ ||
            bytes_[3] != kAoyiMasterId)
        {
            bytes_.erase(bytes_.begin());
            continue;
        }

        const auto total_length =
            7 + static_cast<std::size_t>(bytes_[5]);
        if (bytes_.size() < total_length)
        {
            return std::nullopt;
        }

        std::vector<std::uint8_t> candidate(
            bytes_.begin(),
            bytes_.begin() + static_cast<std::ptrdiff_t>(total_length));
        if (const auto packet = decodePacket(candidate))
        {
            bytes_.erase(
                bytes_.begin(),
                bytes_.begin() + static_cast<std::ptrdiff_t>(total_length));
            return packet;
        }

        bytes_.erase(bytes_.begin());
    }
}

} // namespace robot::ti5::hand
