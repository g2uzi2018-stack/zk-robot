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

        left.commandPositionCsp(3.14159265358979323846 / 2.0);
        expect(transport_ptr->sent_frames.size() == 1,
               "Position CSP command must send exactly one frame");
        const auto &command = transport_ptr->sent_frames.back();
        expect(command.id == 23 && command.data_length == 5 && command.data[0] == 0x44,
               "Position CSP command header mismatch");
        expect(command.data[1] == 0x00 && command.data[2] == 0x00 &&
                   command.data[3] == 0x01 && command.data[4] == 0x00,
               "Position CSP command count encoding mismatch");

        auto malformed = cspFrame(23, 1, 2, 3);
        malformed.data_length = 7;
        transport_ptr->enqueue(malformed);
        transport_ptr->enqueue(cspFrame(23, 0x1234, -2, -65536));
        transport_ptr->enqueue(cspFrame(24, 0x2345, 3, 65536));
        transport_ptr->enqueue(cspFrame(25, 0x3456, 4, 123));
        bus.collectPendingFeedback();

        const auto left_feedback = bus.latestCspFeedback(23);
        const auto right_feedback = bus.latestCspFeedback(24);
        expect(left_feedback && right_feedback,
               "valid feedback for both nodes must be cached");
        expect(left_feedback->current_milliamps == 0x1234 &&
                   left_feedback->speed_raw == -2 &&
                   left_feedback->position_counts == -65536,
               "left CSP feedback decode mismatch");
        expect(right_feedback->current_milliamps == 0x2345 &&
                   right_feedback->position_counts == 65536,
               "right CSP feedback decode mismatch");
        expect(left_feedback->timestamp != std::chrono::steady_clock::time_point{},
               "feedback timestamp must be populated");
        expect(!bus.latestCspFeedback(25),
               "unregistered node feedback must not enter the cache");

        auto malformed_position = positionFrame(23, 1);
        malformed_position.data_length = 4;
        transport_ptr->enqueue(malformed_position);
        bus.collectPendingFeedback();
        expect(bus.latestCspFeedback(23)->position_counts == -65536,
               "malformed position feedback must not overwrite CSP state");

        transport_ptr->on_send = [transport_ptr](const robot::can::CanFrame &frame) {
            if (frame.data_length == 1 && frame.data[0] == 0x08)
            {
                transport_ptr->enqueue(positionFrame(frame.id, 65536));
            }
            else if (frame.data_length == 1 && frame.data[0] == 0x41)
            {
                transport_ptr->enqueue(cspFrame(frame.id, 10, 20, 30));
            }
        };

        const auto queried_position = left.queryPosition();
        expect(queried_position &&
                   std::abs(*queried_position - 3.14159265358979323846 / 2.0) < 1e-12,
               "CanMotor queryPosition must use the shared CanBus response");
        const auto queried_csp = right.queryCspStatus();
        expect(queried_csp && queried_csp->source == robot::ti5::MotorFeedback::Source::Csp &&
                   queried_csp->position_counts == 30,
               "CanMotor queryCspStatus must return the shared CSP response");
        expect(left.latestFeedback() && right.latestFeedback(),
               "motors on one logical bus must share feedback storage");

        expect(std::abs(robot::ti5::speedRawToOutputRadiansPerSecond(101, 101.0) -
                        2.0 * 3.14159265358979323846 / 100.0) < 1e-12,
               "CSP speed conversion mismatch");

        std::cout << "TI5 CanBus and CanMotor tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 CanBus and CanMotor test failed: " << error.what() << '\n';
        return 1;
    }
}
