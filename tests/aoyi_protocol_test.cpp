#include "can/can_frame.hpp"
#include "ti5/hand/aoyi_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

robot::can::CanFrame frameFromBytes(
    const std::uint16_t id,
    const std::vector<std::uint8_t> &bytes)
{
    expect(!bytes.empty() && bytes.size() <= 8, "test frame length is invalid");
    robot::can::CanFrame frame;
    frame.id = id;
    frame.data_length = static_cast<std::uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), frame.data.begin());
    return frame;
}

std::optional<robot::ti5::hand::AoyiPacket> feed(
    robot::ti5::hand::PacketReassembler &reassembler,
    const std::vector<robot::can::CanFrame> &frames,
    const std::chrono::steady_clock::time_point start)
{
    std::optional<robot::ti5::hand::AoyiPacket> result;
    for (std::size_t index = 0; index < frames.size(); ++index)
    {
        const auto packet = reassembler.push(
            frames[index],
            start + std::chrono::milliseconds{static_cast<int>(index)});
        if (packet)
        {
            result = packet;
        }
    }
    return result;
}

} // namespace

int main()
{
    try
    {
        using namespace robot::ti5::hand;

        const auto query = encodePacket(
            70,
            kAoyiGetStatusCommand,
            {kAoyiGetStatusSubcommand});
        expect(
            query == std::vector<std::uint8_t>{
                0x55, 0xAA, 0x46, 0x01, 0x5F, 0x01, 0x80, 0x99},
            "status query packet encoding or LRC is incorrect");

        const auto decoded_query = decodePacket(query);
        expect(
            decoded_query &&
                decoded_query->hand_id == 70 &&
                decoded_query->command == kAoyiGetStatusCommand &&
                decoded_query->payload ==
                    std::vector<std::uint8_t>{kAoyiGetStatusSubcommand},
            "status query packet did not decode");

        auto bad_header = query;
        bad_header[0] = 0x54;
        expect(!decodePacket(bad_header), "malformed header was accepted");

        auto bad_length = query;
        bad_length[5] = 2;
        expect(!decodePacket(bad_length), "invalid packet length was accepted");

        auto bad_lrc = query;
        bad_lrc.back() ^= 0x01;
        expect(!decodePacket(bad_lrc), "invalid LRC was accepted");

        AoyiPacket response;
        response.hand_id = 70;
        response.master_id = kAoyiMasterId;
        response.command = kAoyiGetStatusCommand;
        response.payload.resize(18);
        for (std::size_t index = 0; index < response.payload.size(); ++index)
        {
            response.payload[index] =
                static_cast<std::uint8_t>(index + 1);
        }
        const auto response_bytes = encodePacket(
            response.hand_id,
            response.command,
            response.payload);
        const auto response_frames = fragmentPacket(70, response_bytes);
        expect(
            response_frames.size() == 4,
            "packet was not split into 8-byte frames");
        for (const auto &frame : response_frames)
        {
            expect(frame.id == 70, "fragment CAN ID changed");
            expect(frame.data_length <= 8, "fragment exceeded Classic CAN DLC");
        }

        PacketReassembler reassembler{70, 70, std::chrono::milliseconds{50}};
        const auto start = std::chrono::steady_clock::now();
        const auto reassembled = feed(reassembler, response_frames, start);
        expect(
            reassembled &&
                reassembled->payload == response.payload,
            "multi-frame packet did not reassemble");

        std::vector<std::uint8_t> malformed_stream{
            0x12, 0x55, 0x00, 0x55};
        malformed_stream.insert(
            malformed_stream.end(),
            query.begin(),
            query.end());
        PacketReassembler header_resync{70, 70, std::chrono::milliseconds{50}};
        const auto header_resync_result = feed(
            header_resync,
            fragmentPacket(70, malformed_stream),
            start);
        expect(
            header_resync_result &&
                header_resync_result->command == kAoyiGetStatusCommand,
            "reassembler did not recover from malformed header");

        std::vector<std::uint8_t> invalid_then_valid = bad_lrc;
        invalid_then_valid.insert(
            invalid_then_valid.end(),
            query.begin(),
            query.end());
        PacketReassembler lrc_resync{70, 70, std::chrono::milliseconds{50}};
        const auto lrc_resync_result = feed(
            lrc_resync,
            fragmentPacket(70, invalid_then_valid),
            start);
        expect(
            lrc_resync_result &&
                lrc_resync_result->hand_id == 70,
            "reassembler did not recover after invalid LRC");

        PacketReassembler timeout_resync{70, 70, std::chrono::milliseconds{20}};
        const auto partial = frameFromBytes(70, {0x55, 0xAA, 0x46, 0x01});
        static_cast<void>(timeout_resync.push(partial, start));
        const auto timeout_result = feed(
            timeout_resync,
            fragmentPacket(70, query),
            start + std::chrono::milliseconds{100});
        expect(
            timeout_result &&
                timeout_result->command == kAoyiGetStatusCommand,
            "reassembler did not reset after inter-frame timeout");

        robot::can::CanFrame invalid_frame;
        invalid_frame.id = 70;
        invalid_frame.data_length = 9;
        expect(
            !timeout_resync.push(invalid_frame, start),
            "invalid CAN frame length was accepted");

        std::cout << "Aoyi protocol tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Aoyi protocol test failed: " << error.what() << '\n';
        return 1;
    }
}
