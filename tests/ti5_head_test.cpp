#include "ti5/can/encoder_conversion.hpp"
#include "ti5/head/head.hpp"

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

class HeadSimulation final
{
public:
    explicit HeadSimulation(FakeTransport &transport)
        : transport_(transport)
    {
        const auto limit = robot::ti5::radiansToPositionCounts(
            0.5, kCountsPerRevolution);
        for (std::uint16_t node = 30; node <= 32; ++node)
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
                transport_.enqueue(valueFrame(frame.id, 0x0A, 0));
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
            mode_.at(frame.id) = 8;
            transport_.enqueue(cspFrame(frame.id, position_.at(frame.id)));
        }
    }

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
    config.physical_joint.bus = "head";
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

std::vector<robot::ti5::JointConfig> headConfigs()
{
    return {
        jointConfig("neck_roll", 32),
        jointConfig("neck_yaw", 30),
        jointConfig("neck_pitch", 31)};
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
        HeadSimulation simulation(*transport_pointer);
        auto bus = std::make_unique<robot::ti5::CanBus>(
            std::move(transport));

        robot::ti5::HeadOptions options;
        options.control_period = std::chrono::milliseconds{0};
        options.inter_frame_gap = std::chrono::microseconds{0};
        options.position_control_start_cycles = 2;
        options.maximum_feedback_age = std::chrono::seconds{1};
        options.stop_inter_joint_gap = std::chrono::milliseconds{0};
        options.stop_settle_time = std::chrono::milliseconds{0};

        robot::ti5::Head head(
            std::move(bus), headConfigs(), options);
        expect(head.jointNames()[0] == "neck_yaw" &&
                   head.jointNames()[2] == "neck_roll" &&
                   head.joint(0).nodeId() == 30 &&
                   head.joint(2).nodeId() == 32,
               "Head did not assemble joints by semantic name");

        head.prepare();
        head.startPositionControlAtCurrentPosition();
        expect(head.controlState() ==
                   robot::ti5::HeadControlState::PositionControlActive,
               "Head did not establish position control");
        const auto state = head.readState();
        expect(state.all_positions_available &&
                   state.all_csp_feedback_fresh,
               "Head did not aggregate three-axis feedback");

        robot::ti5::Head::JointValues invalid{};
        invalid[1] = 0.55;
        const auto before = countCommand(transport_pointer->sent, 0x44);
        expectThrow<std::out_of_range>(
            [&head, &invalid]() { head.commandPositionsCsp(invalid); },
            "Head accepted a target outside driver limits");
        expect(countCommand(transport_pointer->sent, 0x44) == before,
               "Head sent a partial invalid target batch");

        robot::ti5::Head::JointValues target{};
        target.fill(0.1);
        head.commandPositionsCsp(target);
        expect(countCommand(transport_pointer->sent, 0x44) == before + 3,
               "Head valid target did not send three frames");

        const auto stop_start = transport_pointer->sent.size();
        head.requestStopModeAndConfirm();
        expect(head.controlState() == robot::ti5::HeadControlState::Stopped,
               "Head STOP was not confirmed");
        for (std::size_t index = 0; index < 3; ++index)
        {
            const auto &frame = transport_pointer->sent[stop_start + index];
            expect(frame.data_length == 1 && frame.data[0] == 0x02 &&
                       frame.id == static_cast<std::uint16_t>(32 - index),
                   "Head STOP order mismatch");
        }

        auto missing = headConfigs();
        missing.pop_back();
        expectThrow<std::invalid_argument>(
            [&missing]()
            {
                auto fake = std::make_unique<FakeTransport>();
                auto invalid_bus = std::make_unique<robot::ti5::CanBus>(
                    std::move(fake));
                robot::ti5::Head invalid_head(
                    std::move(invalid_bus), missing);
            },
            "Head accepted an incomplete joint set");

        std::cout << "TI5 Head tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 Head test failed: " << error.what() << '\n';
        return 1;
    }
}
