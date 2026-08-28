#include "ti5/controller/hand_controller.hpp"
#include "ti5/hand/hand.hpp"
#include "ti5/hand/aoyi_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <deque>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
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

template <typename Exception, typename Function>
void expectThrow(Function &&function, const std::string &message)
{
    try
    {
        function();
    }
    catch (const Exception &)
    {
        return;
    }
    throw std::runtime_error(message);
}

class FakeHandTransport final : public robot::ti5::hand::HandTransport
{
public:
    explicit FakeHandTransport(const std::uint8_t hand_id)
        : hand_id_(hand_id),
          request_reassembler_(
              hand_id,
              hand_id,
              std::chrono::milliseconds{100})
    {
    }

    void send(const robot::can::CanFrame &frame) override
    {
        sent.push_back(frame);
        const auto packet = request_reassembler_.push(frame);
        if (!packet ||
            packet->command !=
                robot::ti5::hand::kAoyiGetStatusCommand)
        {
            return;
        }

        std::vector<std::uint8_t> payload;
        for (std::uint16_t index = 0;
             index < robot::ti5::hand::kAoyiChannelCount;
             ++index)
        {
            const auto position = static_cast<std::uint16_t>(100 + index);
            payload.push_back(static_cast<std::uint8_t>(position & 0xFFU));
            payload.push_back(static_cast<std::uint8_t>(
                (position >> 8U) & 0xFFU));
            payload.push_back(0xA0);
        }
        for (std::uint8_t index = 0;
             index < robot::ti5::hand::kAoyiChannelCount;
             ++index)
        {
            payload.push_back(static_cast<std::uint8_t>(10 + index));
        }

        const auto bytes = robot::ti5::hand::encodePacket(
            hand_id_,
            robot::ti5::hand::kAoyiGetStatusCommand,
            payload);
        for (const auto &response :
             robot::ti5::hand::fragmentPacket(hand_id_, bytes))
        {
            incoming.push_back(response);
        }
    }

    std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds) override
    {
        if (incoming.empty())
        {
            return std::nullopt;
        }
        const auto frame = incoming.front();
        incoming.pop_front();
        return frame;
    }

    std::vector<robot::can::CanFrame> sent;

private:
    std::uint8_t hand_id_{0};
    robot::ti5::hand::PacketReassembler request_reassembler_;
    std::deque<robot::can::CanFrame> incoming;
};

robot::ti5::hand::HandSideConfig leftConfig(const bool control_allowed)
{
    robot::ti5::hand::HandSideConfig config;
    config.name = "left_hand";
    config.protocol = "aoyi_hand";
    config.controller_node_id = 70;
    config.protocol_verified = control_allowed;
    config.control_enabled = control_allowed;
    config.discovery_enabled = true;
    return config;
}

std::optional<robot::ti5::hand::AoyiPacket> decodeSentPacket(
    const std::vector<robot::can::CanFrame> &frames,
    const std::uint8_t hand_id)
{
    robot::ti5::hand::PacketReassembler reassembler(
        hand_id,
        hand_id,
        std::chrono::milliseconds{100});
    for (const auto &frame : frames)
    {
        if (const auto packet = reassembler.push(frame))
        {
            if (packet->command ==
                robot::ti5::hand::kAoyiSetPositionsCommand)
            {
                return packet;
            }
        }
    }
    return std::nullopt;
}

std::size_t countSentCommand(
    const std::vector<robot::can::CanFrame> &frames,
    const std::uint8_t hand_id,
    const std::uint8_t command)
{
    robot::ti5::hand::PacketReassembler reassembler(
        hand_id,
        hand_id,
        std::chrono::milliseconds{100});
    std::size_t count = 0;
    for (const auto &frame : frames)
    {
        if (const auto packet = reassembler.push(frame);
            packet && packet->command == command)
        {
            ++count;
        }
    }
    return count;
}

} // namespace

int main()
{
    try
    {
        using namespace robot::ti5;

        auto read_only_transport =
            std::make_unique<FakeHandTransport>(70);
        auto *read_only_pointer = read_only_transport.get();
        Hand read_only(
            HandSide::Left,
            leftConfig(false),
            std::move(read_only_transport),
            std::chrono::milliseconds{100});
        expect(read_only.name() == "left_hand" &&
                   read_only.controllerNodeId() == 70 &&
                   !read_only.controlAllowed(),
               "read-only Hand identity or control guard mismatch");

        const auto state = read_only.readState();
        expect(state && state->positions_raw[0] == 100 &&
                   state->positions_raw[5] == 105 &&
                   state->forces_raw &&
                   (*state->forces_raw)[0] == 10 &&
                   (*state->forces_raw)[5] == 15,
               "Hand status decode failed");

        Hand::PositionValues positions{};
        Hand::SpeedValues speeds{};
        positions.fill(500);
        speeds.fill(20);

        HandController read_only_controller(read_only);
        expectThrow<std::logic_error>(
            [&]() { read_only_controller.start(speeds); },
            "read-only HandController started control");
        expect(read_only_controller.state() ==
                   HandController::ControlState::Idle,
               "configuration rejection changed HandController state");

        const auto sent_before_rejection = read_only_pointer->sent.size();
        expectThrow<std::logic_error>(
            [&]() { read_only.commandPositionsRaw(positions, speeds); },
            "read-only Hand accepted a motion command");
        expect(read_only_pointer->sent.size() == sent_before_rejection,
               "disabled Hand sent frames before rejecting control");

        auto enabled_transport = std::make_unique<FakeHandTransport>(70);
        auto *enabled_pointer = enabled_transport.get();
        Hand enabled(
            HandSide::Left,
            leftConfig(true),
            std::move(enabled_transport),
            std::chrono::milliseconds{100});
        HandController controller(enabled);
        controller.start(speeds);
        expect(controller.state() ==
                   HandController::ControlState::Running &&
                   controller.targetReachedRaw(0),
               "HandController did not capture current position on start");
        const auto frames_before_target = enabled_pointer->sent.size();
        controller.setTarget(positions, speeds);
        expect(enabled_pointer->sent.size() == frames_before_target,
               "HandController setTarget sent before update");
        controller.update();
        const auto packet = decodeSentPacket(enabled_pointer->sent, 70);
        expect(packet &&
                   packet->command ==
                       hand::kAoyiSetPositionsCommand &&
                   packet->payload.size() == 18 &&
                   packet->payload[0] == 0xF4 &&
                   packet->payload[1] == 0x01 &&
                   packet->payload[2] == 20,
               "HandController 0x50 position packet mismatch");
        controller.pause();
        expect(controller.state() == HandController::ControlState::Idle,
               "HandController pause did not return to Idle");
        const auto position_commands_before_idle_update = countSentCommand(
            enabled_pointer->sent,
            70,
            hand::kAoyiSetPositionsCommand);
        controller.update();
        expect(countSentCommand(
                   enabled_pointer->sent,
                   70,
                   hand::kAoyiSetPositionsCommand) ==
                   position_commands_before_idle_update,
               "paused HandController continued sending 0x50");

        auto wrong_transport = std::make_unique<FakeHandTransport>(70);
        expectThrow<std::invalid_argument>(
            [&]()
            {
                Hand wrong_side(
                    HandSide::Right,
                    leftConfig(false),
                    std::move(wrong_transport),
                    std::chrono::milliseconds{100});
            },
            "Hand accepted a mismatched side config");

        std::cout << "TI5 Hand tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 Hand test failed: " << error.what() << '\n';
        return 1;
    }
}
