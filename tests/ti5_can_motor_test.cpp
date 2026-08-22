#include "ti5/can/can_bus.hpp"
#include "ti5/can/encoder_conversion.hpp"
#include "ti5/motor/can_motor.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
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

    class FakeTransport final : public robot::ti5::CanBusTransport
    {
    public:
        void send(const robot::can::CanFrame &frame) override
        {
            sent_frames.push_back(frame);
            if (on_send)
            {
                on_send(frame);
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

        void enqueue(const robot::can::CanFrame &frame)
        {
            incoming.push_back(frame);
        }

        std::vector<robot::can::CanFrame> sent_frames;
        std::function<void(const robot::can::CanFrame &)> on_send;

    private:
        std::deque<robot::can::CanFrame> incoming;
    };

    robot::ti5::CanMotorConfig motorConfig(const std::uint16_t node_id)
    {
        robot::ti5::CanMotorConfig config;
        config.node_id = node_id;
        config.unit = robot::ti5::JointUnit::Radian;
        config.encoder.type = "dual";
        config.encoder.position_reference = "output";
        config.encoder.counts_per_output_revolution = 262144;
        config.encoder.gear_ratio = 101.0;
        return config;
    }

    robot::can::CanFrame cspFrame(
        const std::uint16_t node_id,
        const std::int16_t current,
        const std::int16_t speed,
        const std::int32_t position)
    {
        robot::can::CanFrame frame{};
        frame.id = node_id;
        frame.data_length = 8;
        const auto current_raw = static_cast<std::uint16_t>(current);
        const auto speed_raw = static_cast<std::uint16_t>(speed);
        const auto position_raw = static_cast<std::uint32_t>(position);
        frame.data = {
            static_cast<std::uint8_t>(current_raw & 0xFFU),
            static_cast<std::uint8_t>((current_raw >> 8U) & 0xFFU),
            static_cast<std::uint8_t>(speed_raw & 0xFFU),
            static_cast<std::uint8_t>((speed_raw >> 8U) & 0xFFU),
            static_cast<std::uint8_t>(position_raw & 0xFFU),
            static_cast<std::uint8_t>((position_raw >> 8U) & 0xFFU),
            static_cast<std::uint8_t>((position_raw >> 16U) & 0xFFU),
            static_cast<std::uint8_t>((position_raw >> 24U) & 0xFFU)};
        return frame;
    }

    robot::can::CanFrame positionFrame(
        const std::uint16_t node_id,
        const std::int32_t position)
    {
        const auto raw = static_cast<std::uint32_t>(position);
        robot::can::CanFrame frame{};
        frame.id = node_id;
        frame.data_length = 5;
        frame.data = {
            0x08,
            static_cast<std::uint8_t>(raw & 0xFFU),
            static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
            static_cast<std::uint8_t>((raw >> 16U) & 0xFFU),
            static_cast<std::uint8_t>((raw >> 24U) & 0xFFU),
            0,
            0,
            0};
        return frame;
    }
}

int main()
{
    try
    {
        auto transport = std::make_unique<FakeTransport>();
        auto *transport_ptr = transport.get();
        robot::ti5::CanBus bus(std::move(transport));
        robot::ti5::CanMotor left(motorConfig(23), bus);
        robot::ti5::CanMotor right(motorConfig(24), bus);

        expect(left.nodeId() == 23 && right.nodeId() == 24,
               "motors must retain their own node IDs");
        expect(bus.feedbackFormat(23) == robot::ti5::FeedbackFormat::Csp &&
                   bus.feedbackFormat(24) == robot::ti5::FeedbackFormat::Csp,
               "CanMotor must explicitly register CSP parser contexts");

        left.commandPositionCsp(3.14159265358979323846 / 2.0);
        expect(transport_ptr->sent_frames.size() == 1,
               "Position CSP command must send exactly one frame");
        const auto &command = transport_ptr->sent_frames.back();
        expect(command.id == 23 && command.data_length == 5 &&
                   command.data[0] == 0x44,
               "Position CSP command header mismatch");
        expect(command.data[1] == 0x00 && command.data[2] == 0x00 &&
                   command.data[3] == 0x01 && command.data[4] == 0x00,
               "Position CSP command count encoding mismatch");

        // A CSP frame fills all three fields of the aggregate state.
        transport_ptr->enqueue(cspFrame(23, 10, 20, 100));
        bus.collectPendingFeedback();
        const auto first_state = bus.latestState(23);
        expect(first_state && first_state->position_counts &&
                   first_state->speed_raw && first_state->current_milliamps,
               "CSP feedback must populate all MotorState fields");
        expect(first_state->position_counts->value == 100 &&
                   first_state->speed_raw->value == 20 &&
                   first_state->current_milliamps->value == 10,
               "initial CSP MotorState values mismatch");
        const auto speed_timestamp = first_state->speed_raw->timestamp;
        const auto current_timestamp = first_state->current_milliamps->timestamp;
        const auto first_sequence = first_state->update_sequence;

        // 0x08 only supplies position and must merge into the existing state.
        transport_ptr->enqueue(positionFrame(23, 200));
        bus.collectPendingFeedback();
        const auto merged_state = bus.latestState(23);
        expect(merged_state && merged_state->position_counts &&
                   merged_state->speed_raw && merged_state->current_milliamps,
               "0x08 must preserve existing CSP fields");
        expect(merged_state->position_counts->value == 200 &&
                   merged_state->speed_raw->value == 20 &&
                   merged_state->current_milliamps->value == 10,
               "0x08 aggregate merge mismatch");
        expect(merged_state->update_sequence == first_sequence + 1 &&
                   merged_state->speed_raw->timestamp == speed_timestamp &&
                   merged_state->current_milliamps->timestamp == current_timestamp &&
                   merged_state->position_counts->timestamp >=
                       first_state->position_counts->timestamp,
               "field timestamps or update sequence were merged incorrectly");

        // A second node has independent state and raw CSP storage.
        transport_ptr->enqueue(cspFrame(24, 30, -4, 600));
        bus.collectPendingFeedback();
        const auto right_state = bus.latestState(24);
        expect(right_state && right_state->position_counts &&
                   right_state->position_counts->value == 600,
               "second node CSP state missing");
        expect(bus.latestState(23)->position_counts->value == 200,
               "different node feedback must not overwrite left state");
        const auto right_csp = bus.latestCspFeedback(24);
        expect(right_csp && right_csp->current_milliamps == 30 &&
                   right_csp->speed_raw == -4 &&
                   right_csp->position_counts == 600,
               "CSP raw feedback cache mismatch");

        // Malformed feedback is retained as raw/unsupported and cannot
        // overwrite a previously valid aggregate state.
        auto malformed_csp = cspFrame(23, 1, 2, 3);
        malformed_csp.data_length = 7;
        transport_ptr->enqueue(malformed_csp);
        bus.collectPendingFeedback();
        expect(bus.latestState(23)->position_counts->value == 200 &&
                   bus.unsupportedFeedbackCount(23) == 1,
               "malformed CSP feedback must not overwrite state");

        // Unknown and PT contexts must not send their DLC=8 payloads through
        // the CSP parser.  The frame is retained for future decoders.
        bus.registerNode(25, robot::ti5::FeedbackFormat::Unknown);
        bus.registerNode(26, robot::ti5::FeedbackFormat::Pt);
        const auto unknown_frame = cspFrame(25, 0x3456, 4, 123);
        const auto pt_frame = cspFrame(26, 0x4567, 5, 456);
        transport_ptr->enqueue(unknown_frame);
        transport_ptr->enqueue(pt_frame);
        bus.collectPendingFeedback();
        expect(!bus.latestState(25) && !bus.latestCspFeedback(25) &&
                   bus.unsupportedFeedbackCount(25) == 1,
               "Unknown node mode must not be decoded as CSP");
        expect(!bus.latestState(26) && !bus.latestCspFeedback(26) &&
                   bus.unsupportedFeedbackCount(26) == 1,
               "PT node mode must not be decoded as CSP");
        expect(bus.latestRawFrame(25) &&
                   bus.latestRawFrame(25)->frame.data == unknown_frame.data &&
                   bus.latestRawFrame(26) &&
                   bus.latestRawFrame(26)->frame.data == pt_frame.data,
               "unsupported node frames must retain raw payloads");

        const auto sent_before_rejected_queries = transport_ptr->sent_frames.size();
        expect(!bus.queryCsp(25, std::chrono::milliseconds{0}) &&
                   !bus.queryCsp(26, std::chrono::milliseconds{0}) &&
                   transport_ptr->sent_frames.size() == sent_before_rejected_queries,
               "queryCsp must reject Unknown/PT parser contexts");

        transport_ptr->on_send = [transport_ptr](const robot::can::CanFrame &frame) {
            if (frame.data_length == 1 && frame.data[0] == 0x08)
            {
                transport_ptr->enqueue(positionFrame(frame.id, 65536));
            }
            else if (frame.data_length == 1 && frame.data[0] == 0x41)
            {
                transport_ptr->enqueue(cspFrame(frame.id, 40, 50, 300));
            }
        };

        const auto queried_position = left.queryPosition();
        expect(queried_position &&
                   std::abs(*queried_position -
                            3.14159265358979323846 / 2.0) < 1e-12,
               "CanMotor queryPosition must use the shared CanBus response");
        expect(bus.latestState(23)->position_counts->value == 65536,
               "queried 0x08 position must merge into MotorState");

        // 0x41 feedback and the feedback following 0x44 have one state path.
        const auto queried_csp = left.queryCspStatus();
        expect(queried_csp && queried_csp->current_milliamps == 40 &&
                   queried_csp->speed_raw == 50 &&
                   queried_csp->position_counts == 300,
               "CanMotor queryCspStatus must return the CSP response");
        left.commandPositionCsp(1.0);
        transport_ptr->enqueue(cspFrame(23, 41, 51, 400));
        bus.collectPendingFeedback();
        const auto after_position_command = left.latestState();
        expect(after_position_command &&
                   after_position_command->position_counts->value == 400 &&
                   after_position_command->speed_raw->value == 51 &&
                   after_position_command->current_milliamps->value == 41,
               "0x41 and 0x44 feedback must merge into one MotorState");

        expect(std::abs(robot::ti5::speedRawToOutputRadiansPerSecond(101, 101.0) -
                        2.0 * 3.14159265358979323846 / 100.0) < 1e-12,
               "CSP speed conversion mismatch");

        std::cout << "TI5 CanBus and CanMotor tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 CanBus and CanMotor test failed: " << error.what() <<
                     '\n';
        return 1;
    }
}
