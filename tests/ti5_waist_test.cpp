#include "ti5/can/encoder_conversion.hpp"
#include "ti5/controller/waist_controller.hpp"
#include "ti5/waist/waist.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <thread>

namespace
{

constexpr std::int32_t kCountsPerRevolution = 262144;

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

robot::can::CanFrame valueFrame(const std::uint16_t node_id,
                                const std::uint8_t command,
                                const std::int32_t value)
{
    const auto raw = static_cast<std::uint32_t>(value);
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 5;
    frame.data = {
        command,
        static_cast<std::uint8_t>(raw & 0xFFU),
        static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
        static_cast<std::uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((raw >> 24U) & 0xFFU),
        0,
        0,
        0};
    return frame;
}

robot::can::CanFrame cspFrame(const std::uint16_t node_id,
                              const std::int32_t position)
{
    const auto raw = static_cast<std::uint32_t>(position);
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 8;
    frame.data = {
        0xE8,
        0x03,
        101,
        0,
        static_cast<std::uint8_t>(raw & 0xFFU),
        static_cast<std::uint8_t>((raw >> 8U) & 0xFFU),
        static_cast<std::uint8_t>((raw >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((raw >> 24U) & 0xFFU)};
    return frame;
}

class FakeTransport final : public robot::ti5::CanBusTransport
{
public:
    void send(const robot::can::CanFrame &frame) override
    {
        sent.push_back(frame);
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

    std::vector<robot::can::CanFrame> sent;
    std::function<void(const robot::can::CanFrame &)> on_send;

private:
    std::deque<robot::can::CanFrame> incoming;
};

class WaistSimulation final
{
public:
    explicit WaistSimulation(FakeTransport &transport)
        : transport_(transport)
    {
        const auto limit = robot::ti5::radiansToPositionCounts(
            0.5, kCountsPerRevolution);
        for (std::uint16_t node = 1; node <= 5; ++node)
        {
            position_[node] = 0;
            mode_[node] = 0;
            limit_[node] = limit;
        }
        transport_.on_send = [this](const robot::can::CanFrame &frame)
        {
            handle(frame);
        };
    }

private:
    void handle(const robot::can::CanFrame &frame)
    {
        if (frame.data_length == 1)
        {
            switch (frame.data[0])
            {
            case 0x1A:
                transport_.enqueue(valueFrame(
                    frame.id, 0x1A, limit_.at(frame.id)));
                return;
            case 0x1B:
                transport_.enqueue(valueFrame(
                    frame.id, 0x1B, -limit_.at(frame.id)));
                return;
            case 0x08:
                transport_.enqueue(valueFrame(
                    frame.id, 0x08, position_.at(frame.id)));
                return;
            case 0x03:
                transport_.enqueue(valueFrame(
                    frame.id,
                    0x03,
                    static_cast<std::int32_t>(mode_.at(frame.id))));
                return;
            case 0x0A:
                if (!drop_status_) transport_.enqueue(valueFrame(frame.id, 0x0A, fault_));
                return;
            case 0x02:
                mode_.at(frame.id) = 0;
                return;
            default:
                return;
            }
        }
        if (frame.data_length == 5 && frame.data[0] == 0x44)
        {
            const auto raw =
                static_cast<std::uint32_t>(frame.data[1]) |
                (static_cast<std::uint32_t>(frame.data[2]) << 8U) |
                (static_cast<std::uint32_t>(frame.data[3]) << 16U) |
                (static_cast<std::uint32_t>(frame.data[4]) << 24U);
            position_.at(frame.id) = static_cast<std::int32_t>(raw);
            if (accept_mode_) mode_.at(frame.id) = 8;
            if (!drop_csp_) transport_.enqueue(cspFrame(frame.id, position_.at(frame.id)));
        }
    }

public:
    int fault_{0};
    bool drop_status_{false};
    bool accept_mode_{true};
    bool drop_csp_{false};
    FakeTransport &transport_;
    std::map<std::uint16_t, std::int32_t> position_;
    std::map<std::uint16_t, std::uint32_t> mode_;
    std::map<std::uint16_t, std::int32_t> limit_;
};

robot::ti5::JointConfig jointConfig(const std::string &name,
                                    const std::uint16_t node_id)
{
    robot::ti5::JointConfig config;
    config.physical_joint.name = name;
    config.physical_joint.physical_name = name;
    config.physical_joint.bus = "waist_fold";
    config.physical_joint.motor.node_id = node_id;
    config.physical_joint.motor.unit = robot::ti5::JointUnit::Radian;
    config.physical_joint.motor.encoder.type = "dual";
    config.physical_joint.motor.encoder.position_reference = "output";
    config.physical_joint.motor.encoder.counts_per_output_revolution =
        kCountsPerRevolution;
    config.physical_joint.motor.encoder.gear_ratio = 101.0;
    config.motor_position_limits = {-0.6, 0.6, false};
    config.coordinate_transform = {1.0, 0.0};
    return config;
}

std::vector<robot::ti5::JointConfig> waistConfigs()
{
    return {
        jointConfig("fold_r", 5),
        jointConfig("waist_yaw", 1),
        jointConfig("fold_p1", 4),
        jointConfig("fold_p3", 2),
        jointConfig("fold_p2", 3)};
}

std::size_t countCommand(const std::vector<robot::can::CanFrame> &frames,
                         const std::uint8_t command)
{
    return static_cast<std::size_t>(std::count_if(
        frames.begin(),
        frames.end(),
        [command](const auto &frame)
        {
            return frame.data_length > 0 && frame.data[0] == command;
        }));
}

} // namespace

int main()
{
    try
    {
        auto transport = std::make_unique<FakeTransport>();
        auto *transport_pointer = transport.get();
        WaistSimulation simulation(*transport_pointer);
        auto bus = std::make_unique<robot::ti5::CanBus>(
            std::move(transport));

        robot::ti5::WaistOptions options;
        options.control_period = std::chrono::milliseconds{0};
        options.inter_frame_gap = std::chrono::microseconds{0};
        options.position_control_start_cycles = 2;
        options.maximum_feedback_age = std::chrono::seconds{1};

        robot::ti5::Waist waist(
            std::move(bus), waistConfigs(), options);
        expect(waist.jointNames()[0] == "waist_yaw" &&
                   waist.jointNames()[2] == "fold_p2" &&
                   waist.joint(0).nodeId() == 1 &&
                   waist.joint(2).nodeId() == 3,
               "Waist did not assemble joints by semantic name");

        waist.prepare();
        const auto clear_fault_before = countCommand(
            transport_pointer->sent, 0x0B);
        waist.clearFault();
        expect(countCommand(transport_pointer->sent, 0x0B) ==
                   clear_fault_before + 5,
               "Waist clearFault did not send one request per joint");
        waist.startPositionControlAtCurrentPosition();
        expect(waist.controlState() ==
                   robot::ti5::WaistControlState::PositionControlActive,
               "Waist did not establish position control");

        robot::ti5::WaistController controller(waist);
        const auto before_controller_start =
            countCommand(transport_pointer->sent, 0x44);
        controller.start();
        expect(controller.state() ==
                   robot::ti5::WaistController::ControlState::Running &&
                   controller.targetReached(1e-9) &&
                   countCommand(transport_pointer->sent, 0x44) ==
                       before_controller_start,
               "WaistController did not start from feedback without sending");
        const auto state = waist.readState();
        expect(state.all_positions_available &&
                   state.all_csp_feedback_fresh,
               "Waist did not aggregate five-axis feedback");
        const auto positions = waist.readPositions();
        expect(positions[0].has_value() && positions[1].has_value() &&
                   positions[2].has_value() && positions[3].has_value() &&
                   positions[4].has_value(),
               "Waist readPositions did not expose all axes");

        robot::ti5::Waist::JointValues invalid{};
        invalid[1] = 0.55;
        const auto before = countCommand(transport_pointer->sent, 0x44);
        expectThrow<std::out_of_range>(
            [&controller, &invalid]() { controller.setTarget(invalid); },
            "WaistController accepted a target outside driver limits");
        expect(countCommand(transport_pointer->sent, 0x44) == before,
               "WaistController sent before rejecting an invalid target");

        robot::ti5::Waist::JointValues target{};
        target.fill(0.1);
        controller.setTarget(target);
        expect(countCommand(transport_pointer->sent, 0x44) == before,
               "WaistController setTarget sent before update");
        controller.update();
        expect(countCommand(transport_pointer->sent, 0x44) == before + 5,
               "WaistController update did not send five frames");
        expect(controller.targetReached(5e-5),
               "WaistController did not retain updated feedback");
        controller.holdCurrentPosition();

        const auto before_failure = countCommand(transport_pointer->sent, 0x44);
        simulation.mode_[3] = 0;
        expectThrow<std::runtime_error>([&]() { controller.update(); },
            "Waist accepted mode loss hidden by cached mode 8");
        expect(countCommand(transport_pointer->sent, 0x44) == before_failure,
            "Waist sent motion before checking all live modes");
        expect(countCommand(transport_pointer->sent, 0x02) == 0,
            "Waist emitted STOP on failure");
        auto missing = waistConfigs();
        missing.pop_back();
        expectThrow<std::invalid_argument>(
            [&missing]()
            {
                auto fake = std::make_unique<FakeTransport>();
                auto invalid_bus = std::make_unique<robot::ti5::CanBus>(
                    std::move(fake));
                robot::ti5::Waist invalid_waist(
                    std::move(invalid_bus), missing);
            },
            "Waist accepted an incomplete joint set");

        // Every failure must reject further motion without STOP or mode writes.
        for (int scenario = 0; scenario < 7; ++scenario)
        {
            auto fake = std::make_unique<FakeTransport>();
            auto *wire = fake.get();
            WaistSimulation sim(*wire);
            auto owned_bus = std::make_unique<robot::ti5::CanBus>(std::move(fake));
            robot::ti5::Waist protected_waist(std::move(owned_bus), waistConfigs(), options);
            protected_waist.prepare();
            if (scenario == 0 || scenario == 1)
            {
                if (scenario == 0) sim.drop_csp_ = true;
                else sim.accept_mode_ = false;
                expectThrow<std::runtime_error>([&]() {
                    protected_waist.startPositionControlAtCurrentPosition();
                }, "Waist accepted startup without fresh CSP/mode 8");
            }
            else
            {
                protected_waist.startPositionControlAtCurrentPosition();
                const auto count = countCommand(wire->sent, 0x44);
                if (scenario == 2) sim.fault_ = 0x20000;
                if (scenario == 3) sim.drop_status_ = true;
                if (scenario == 4) sim.mode_[5] = 2;
                if (scenario == 5) std::this_thread::sleep_for(std::chrono::milliseconds{1100});
                if (scenario == 6) wire->on_send = [](const auto &) {
                    throw std::runtime_error("simulated transmit failure");
                };
                expectThrow<std::runtime_error>([&]() {
                    protected_waist.commandPositionsCsp({});
                }, "Waist accepted fault, missing status, wrong mode or stale feedback");
                expect(countCommand(wire->sent, 0x44) == count,
                    "Waist emitted a target after failed preflight");
            }
            expect(protected_waist.controlState() == robot::ti5::WaistControlState::Failed,
                "Waist failed to latch failure");
            for (const auto &frame : wire->sent)
            {
                expect((frame.data_length == 5 && frame.data[0] == 0x44) ||
                       (frame.data_length == 1 && (frame.data[0] == 0x03 ||
                        frame.data[0] == 0x0A || frame.data[0] == 0x08 ||
                        frame.data[0] == 0x0B ||
                        frame.data[0] == 0x1A || frame.data[0] == 0x1B)),
                       "Waist emitted a command outside the read/position allowlist");
            }
        }

        std::cout << "TI5 Waist tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 Waist test failed: " << error.what() << '\n';
        return 1;
    }
}
