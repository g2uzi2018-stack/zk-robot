#include "ti5/can/encoder_conversion.hpp"
#include "ti5/joint/joint.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

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

robot::can::CanFrame int32Frame(const std::uint16_t node_id,
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
                              const std::int16_t speed,
                              const std::int32_t position)
{
    const auto speed_raw = static_cast<std::uint16_t>(speed);
    const auto position_raw = static_cast<std::uint32_t>(position);
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 8;
    frame.data = {
        0,
        0,
        static_cast<std::uint8_t>(speed_raw & 0xFFU),
        static_cast<std::uint8_t>((speed_raw >> 8U) & 0xFFU),
        static_cast<std::uint8_t>(position_raw & 0xFFU),
        static_cast<std::uint8_t>((position_raw >> 8U) & 0xFFU),
        static_cast<std::uint8_t>((position_raw >> 16U) & 0xFFU),
        static_cast<std::uint8_t>((position_raw >> 24U) & 0xFFU)};
    return frame;
}

robot::ti5::JointConfig jointConfig()
{
    robot::ti5::JointConfig config;
    config.physical_joint.name = "left_shoulder_roll";
    config.physical_joint.physical_name = "L_SHOULDER_R";
    config.physical_joint.bus = "left_arm";
    config.physical_joint.motor.node_id = 24;
    config.physical_joint.motor.unit = robot::ti5::JointUnit::Radian;
    config.physical_joint.motor.encoder.type = "dual";
    config.physical_joint.motor.encoder.position_reference = "output";
    config.physical_joint.motor.encoder.counts_per_output_revolution = 262144;
    config.physical_joint.motor.encoder.gear_ratio = 101.0;
    config.motor_position_limits = {-1.4, 1.4, false};
    config.coordinate_transform = {-1.0, kPi / 2.0};
    return config;
}

} // namespace

int main()
{
    try
    {
        auto transport = std::make_unique<FakeTransport>();
        auto *transport_pointer = transport.get();
        robot::ti5::CanBus bus(std::move(transport));
        robot::ti5::Joint joint(jointConfig(), bus);

        expect(joint.name() == "left_shoulder_roll" &&
                   joint.physicalName() == "L_SHOULDER_R" &&
                   joint.busName() == "left_arm" && joint.nodeId() == 24,
               "Joint identity mismatch");
        expect(std::abs(joint.jointToMotorPosition(kPi / 2.0)) < 1e-12 &&
                   std::abs(joint.motorToJointPosition(0.0) - kPi / 2.0) < 1e-12,
               "Joint coordinate transform mismatch");
        expect(std::abs(joint.positionLimits().minimum_rad -
                        (kPi / 2.0 - 1.4)) < 1e-12 &&
                   std::abs(joint.positionLimits().maximum_rad -
                            (kPi / 2.0 + 1.4)) < 1e-12,
               "negative-direction Joint limits were not reordered");

        expectThrow<std::logic_error>(
            [&joint]() { joint.commandPositionCsp(kPi / 2.0); },
            "Joint must reject commands before driver limits are loaded");

        const auto counts_per_revolution =
            jointConfig().physical_joint.motor.encoder.counts_per_output_revolution;
        const auto minimum_counts = robot::ti5::radiansToPositionCounts(
            -1.0, counts_per_revolution);
        const auto maximum_counts = robot::ti5::radiansToPositionCounts(
            1.0, counts_per_revolution);
        const auto position_counts = robot::ti5::radiansToPositionCounts(
            kPi / 4.0, counts_per_revolution);

        transport_pointer->on_send =
            [transport_pointer,
             minimum_counts,
             maximum_counts,
             position_counts](const robot::can::CanFrame &frame)
        {
            if (frame.data_length != 1)
            {
                return;
            }
            if (frame.data[0] == 0x1A)
            {
                transport_pointer->enqueue(
                    int32Frame(frame.id, 0x1A, maximum_counts));
            }
            else if (frame.data[0] == 0x1B)
            {
                transport_pointer->enqueue(
                    int32Frame(frame.id, 0x1B, minimum_counts));
            }
            else if (frame.data[0] == 0x08)
            {
                transport_pointer->enqueue(
                    int32Frame(frame.id, 0x08, position_counts));
            }
        };

        const auto driver_limits = joint.refreshDriverPositionLimits();
        expect(driver_limits && driver_limits->minimum_rad < -0.99 &&
                   driver_limits->maximum_rad > 0.99,
               "Joint failed to load driver position limits");
        const auto command_limits = joint.positionCommandLimits();
        expect(command_limits &&
                   std::abs(command_limits->minimum_rad -
                            (kPi / 2.0 - 1.0)) < 0.0001 &&
                   std::abs(command_limits->maximum_rad -
                            (kPi / 2.0 + 1.0)) < 0.0001,
               "Joint did not expose the transformed command-limit intersection");

        const auto queried_position = joint.queryPosition();
        expect(queried_position &&
                   std::abs(*queried_position - kPi / 4.0) < 0.0001,
               "Joint position query was not transformed to model coordinates");

        expectThrow<std::out_of_range>(
            [&joint]() { joint.commandPositionCsp(0.3); },
            "Joint must reject a target outside driver position limits");
        expectThrow<std::invalid_argument>(
            [&joint]() {
                joint.validatePositionCommand(
                    std::numeric_limits<double>::quiet_NaN());
            },
            "Joint must reject a non-finite position target");

        const auto sent_before_command = transport_pointer->sent_frames.size();
        joint.commandPositionCsp(kPi / 2.0);
        expect(transport_pointer->sent_frames.size() == sent_before_command + 1,
               "valid Joint Position CSP target was not sent");
        const auto &command = transport_pointer->sent_frames.back();
        expect(command.id == 24 && command.data_length == 5 &&
                   command.data[0] == 0x44 && command.data[1] == 0 &&
                   command.data[2] == 0 && command.data[3] == 0 &&
                   command.data[4] == 0,
               "Joint command did not convert model zero offset to motor target");

        joint.requestStopMode();
        expect(transport_pointer->sent_frames.size() ==
                   sent_before_command + 2,
               "Joint STOP-mode request must send one additional frame");
        const auto &stop_request = transport_pointer->sent_frames.back();
        expect(stop_request.id == 24 &&
                   stop_request.data_length == 1 &&
                   stop_request.data[0] == 0x02,
               "Joint STOP-mode request did not delegate to its motor");

        transport_pointer->enqueue(cspFrame(24, 101, 0));
        const auto velocity = joint.readVelocity();
        expect(velocity &&
                   std::abs(*velocity + 2.0 * kPi / 100.0) < 1e-12,
               "Joint velocity direction conversion failed");

        auto invalid = jointConfig();
        invalid.coordinate_transform.direction = 0.5;
        expectThrow<std::invalid_argument>(
            [&invalid]() {
                auto invalid_transport = std::make_unique<FakeTransport>();
                robot::ti5::CanBus invalid_bus(std::move(invalid_transport));
                robot::ti5::Joint invalid_joint(invalid, invalid_bus);
            },
            "Joint must reject a non-unit direction");

        std::cout << "TI5 Joint tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 Joint test failed: " << error.what() << '\n';
        return 1;
    }
}
