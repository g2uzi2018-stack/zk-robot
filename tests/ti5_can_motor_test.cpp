#include "ti5/can/can_bus.hpp"
#include "ti5/motor/can_motor.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <linux/can/error.h>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;

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
        if (fail_send)
        {
            throw std::runtime_error("simulated CAN send failure");
        }
        sent_frames.push_back(frame);
        if (on_send)
        {
            on_send(frame);
        }
    }

    std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds) override
    {
        const auto event = popEvent();
        if (!event ||
            event->kind != robot::can::CanReceiveEventKind::Data)
        {
            return std::nullopt;
        }
        return event->frame;
    }

    std::optional<robot::can::CanReceiveEvent> receiveEvent(
        std::chrono::milliseconds) override
    {
        return popEvent();
    }

    void enqueue(const robot::can::CanFrame &frame)
    {
        robot::can::CanReceiveEvent event;
        event.kind = robot::can::CanReceiveEventKind::Data;
        event.frame = frame;
        event.timestamp = std::chrono::steady_clock::now();
        incoming.push_back(event);
    }

    void enqueueError(const std::uint32_t error_mask,
                      const std::uint8_t controller_flags = 0)
    {
        robot::can::CanReceiveEvent event;
        event.kind = robot::can::CanReceiveEventKind::Error;
        event.error.error_mask = error_mask;
        event.error.data[1] = controller_flags;
        event.timestamp = std::chrono::steady_clock::now();
        incoming.push_back(event);
    }

    bool fail_send{false};
    std::vector<robot::can::CanFrame> sent_frames;
    std::function<void(const robot::can::CanFrame &)> on_send;

private:
    std::optional<robot::can::CanReceiveEvent> popEvent()
    {
        if (incoming.empty())
        {
            return std::nullopt;
        }
        const auto event = incoming.front();
        incoming.pop_front();
        return event;
    }

    std::deque<robot::can::CanReceiveEvent> incoming;
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

robot::can::CanFrame cspFrame(const std::uint16_t node_id,
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

robot::can::CanFrame int32Frame(const std::uint16_t node_id,
                                const std::uint8_t command,
                                const std::uint32_t value)
{
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 5;
    frame.data = {
        command,
        static_cast<std::uint8_t>(value & 0xFFU),
        static_cast<std::uint8_t>((value >> 8U) & 0xFFU),
        static_cast<std::uint8_t>((value >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((value >> 24U) & 0xFFU),
        0,
        0,
        0};
    return frame;
}

} // namespace

int main()
{
    try
    {
        auto transport = std::make_unique<FakeTransport>();
        auto *transport_pointer = transport.get();
        robot::ti5::CanBus bus(std::move(transport));
        robot::ti5::CanMotor motor(motorConfig(23), bus);

        motor.commandPositionCsp(3.14159265358979323846 / 2.0);
        expect(transport_pointer->sent_frames.size() == 1,
               "Position CSP must send one frame");
        const auto &command = transport_pointer->sent_frames.back();
        expect(command.id == 23 && command.data[0] == 0x44 &&
                   command.data[3] == 0x01,
               "Position CSP command encoding mismatch");

        transport_pointer->enqueue(cspFrame(23, 1500, 101, 100));
        bus.collectPendingFeedback();
        const auto first_state = motor.latestState();
        expect(first_state && first_state->csp_update_sequence == 1 &&
                   first_state->position_counts->value == 100 &&
                   first_state->last_csp_feedback_timestamp,
               "CSP state or CSP sequence missing");
        const auto csp_sequence = first_state->csp_update_sequence;
        const auto csp_timestamp = first_state->last_csp_feedback_timestamp;

        // 0x08 可以更新位置，但不能伪装成新的 CSP 反馈。
        transport_pointer->enqueue(int32Frame(23, 0x08, 200U));
        bus.collectPendingFeedback();
        const auto position_updated = motor.latestState();
        expect(position_updated &&
                   position_updated->position_counts->value == 200 &&
                   position_updated->position_query_sequence == 1 &&
                   position_updated->csp_update_sequence == csp_sequence &&
                   position_updated->last_csp_feedback_timestamp == csp_timestamp,
               "0x08 must not advance CSP freshness");

        transport_pointer->on_send =
            [transport_pointer](const robot::can::CanFrame &frame)
        {
            if (frame.data_length != 1)
            {
                return;
            }
            if (frame.data[0] == 0x03)
            {
                transport_pointer->enqueue(int32Frame(frame.id, 0x03, 8U));
            }
            else if (frame.data[0] == 0x0A)
            {
                transport_pointer->enqueue(
                    int32Frame(frame.id, 0x0A, 0x00090042U));
            }
            else if (frame.data[0] == 0x41)
            {
                transport_pointer->enqueue(cspFrame(frame.id, 1500, 101, 300));
            }
        };

        const auto status = motor.queryDriverStatus();
        expect(status && status->run_mode == 8U &&
                   status->fault_bits == 0x00090042U,
               "CanMotor driver status query failed");
        const auto status_state = motor.latestState();
        expect(status_state && status_state->run_mode &&
                   status_state->fault_bits &&
                   status_state->status_update_sequence == 2,
               "driver status was not cached in MotorState");

        const auto csp = motor.queryCspStatus();
        expect(csp && csp->position_counts == 300,
               "CSP query failed");
        const auto velocity = motor.readVelocity();
        const auto current = motor.readCurrentAmps();
        expect(velocity &&
                   std::abs(*velocity -
                            2.0 * 3.14159265358979323846 / 100.0) < 1e-12,
               "motor velocity conversion failed");
        expect(current && std::abs(*current - 1.5) < 1e-12,
               "motor current conversion failed");
        expect(motor.hasFreshCspFeedback(1s),
               "recent CSP feedback must be fresh");

        // CAN 错误事件只更新总线健康状态，不进入任何电机反馈解析器。
        transport_pointer->enqueueError(CAN_ERR_CRTL,
                                        CAN_ERR_CRTL_TX_WARNING);
        bus.collectPendingFeedback();
        expect(bus.health().state == robot::ti5::CanBusState::Warning,
               "CAN warning state not recorded");

        transport_pointer->enqueueError(CAN_ERR_BUSOFF);
        bus.collectPendingFeedback();
        expect(bus.health().state == robot::ti5::CanBusState::BusOff &&
                   bus.health().last_bus_off_timestamp,
               "CAN bus-off state not recorded");

        transport_pointer->enqueueError(CAN_ERR_RESTARTED);
        bus.collectPendingFeedback();
        expect(bus.health().state ==
                   robot::ti5::CanBusState::RestartedAwaitingFeedback,
               "CAN restart must wait for real feedback");

        transport_pointer->enqueue(cspFrame(23, 1, 2, 3));
        bus.collectPendingFeedback();
        expect(bus.health().state == robot::ti5::CanBusState::Healthy,
               "valid feedback after restart must restore link health");
        expect(bus.health().error_frame_count == 3,
               "CAN error frame count mismatch");

        auto failing_transport = std::make_unique<FakeTransport>();
        auto *failing_pointer = failing_transport.get();
        robot::ti5::CanBus failing_bus(std::move(failing_transport), 2);
        failing_pointer->fail_send = true;
        const robot::can::CanFrame test_frame = int32Frame(23, 0x08, 0U);
        for (int attempt = 0; attempt < 2; ++attempt)
        {
            try
            {
                failing_bus.send(test_frame);
            }
            catch (const std::exception &)
            {
            }
        }
        expect(failing_bus.health().send_failure_count == 2 &&
                   failing_bus.health().send_failure_latched,
               "consecutive send failure threshold not latched");

        std::cout << "TI5 CanBus and CanMotor tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 CanBus and CanMotor test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
