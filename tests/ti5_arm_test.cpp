#include "ti5/arm/arm.hpp"
#include "ti5/can/encoder_conversion.hpp"
#include "ti5/controller/arm_controller.hpp"

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

constexpr double kPi = 3.14159265358979323846;
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

std::uint32_t decodeUint32(const robot::can::CanFrame &frame)
{
    return static_cast<std::uint32_t>(frame.data[1]) |
           (static_cast<std::uint32_t>(frame.data[2]) << 8U) |
           (static_cast<std::uint32_t>(frame.data[3]) << 16U) |
           (static_cast<std::uint32_t>(frame.data[4]) << 24U);
}

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

robot::can::CanFrame uint32Frame(const std::uint16_t node_id,
                                 const std::uint8_t command,
                                 const std::uint32_t value)
{
    return int32Frame(node_id, command, static_cast<std::int32_t>(value));
}

robot::can::CanFrame cspFrame(const std::uint16_t node_id,
                              const std::int16_t current_milliamps,
                              const std::int16_t speed,
                              const std::int32_t position)
{
    const auto current_raw =
        static_cast<std::uint16_t>(current_milliamps);
    const auto speed_raw = static_cast<std::uint16_t>(speed);
    const auto position_raw = static_cast<std::uint32_t>(position);
    robot::can::CanFrame frame{};
    frame.id = node_id;
    frame.data_length = 8;
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
        if (incoming_.empty())
        {
            return std::nullopt;
        }
        const auto frame = incoming_.front();
        incoming_.pop_front();
        return frame;
    }

    void enqueue(const robot::can::CanFrame &frame)
    {
        incoming_.push_back(frame);
    }

    std::vector<robot::can::CanFrame> sent_frames;
    std::function<void(const robot::can::CanFrame &)> on_send;

private:
    std::deque<robot::can::CanFrame> incoming_;
};

struct SimulatedArm
{
    explicit SimulatedArm(FakeTransport &transport,
                          const std::uint16_t first_node)
        : transport_(transport)
    {
        const auto limit_counts = robot::ti5::radiansToPositionCounts(
            1.0, kCountsPerRevolution);
        for (std::uint16_t offset = 0; offset < 7; ++offset)
        {
            const auto node_id = static_cast<std::uint16_t>(
                first_node + offset);
            positions_[node_id] = 0;
            run_modes_[node_id] = 0;
            fault_bits_[node_id] = 0;
            minimum_limits_[node_id] = -limit_counts;
            maximum_limits_[node_id] = limit_counts;
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
                transport_.enqueue(int32Frame(
                    frame.id, 0x1A, maximum_limits_.at(frame.id)));
                return;
            case 0x1B:
                transport_.enqueue(int32Frame(
                    frame.id, 0x1B, minimum_limits_.at(frame.id)));
                return;
            case 0x08:
                transport_.enqueue(int32Frame(
                    frame.id, 0x08, positions_.at(frame.id)));
                return;
            case 0x03:
                transport_.enqueue(uint32Frame(
                    frame.id, 0x03, run_modes_.at(frame.id)));
                return;
            case 0x0A:
                transport_.enqueue(uint32Frame(
                    frame.id, 0x0A, fault_bits_.at(frame.id)));
                return;
            case 0x02:
                run_modes_.at(frame.id) = 0;
                return;
            default:
                return;
            }
        }

        if (frame.data_length == 5 && frame.data[0] == 0x44)
        {
            positions_.at(frame.id) =
                static_cast<std::int32_t>(decodeUint32(frame));
            run_modes_.at(frame.id) = 8;
            transport_.enqueue(cspFrame(
                frame.id, 1000, 101, positions_.at(frame.id)));
        }
    }

    FakeTransport &transport_;
    std::map<std::uint16_t, std::int32_t> positions_;
    std::map<std::uint16_t, std::uint32_t> run_modes_;
    std::map<std::uint16_t, std::uint32_t> fault_bits_;
    std::map<std::uint16_t, std::int32_t> minimum_limits_;
    std::map<std::uint16_t, std::int32_t> maximum_limits_;
};

robot::ti5::JointConfig jointConfig(const std::string &name,
                                    const std::string &bus,
                                    const std::uint16_t node_id)
{
    robot::ti5::JointConfig config;
    config.physical_joint.name = name;
    config.physical_joint.physical_name = name;
    config.physical_joint.bus = bus;
    config.physical_joint.motor.node_id = node_id;
    config.physical_joint.motor.unit = robot::ti5::JointUnit::Radian;
    config.physical_joint.motor.encoder.type = "dual";
    config.physical_joint.motor.encoder.position_reference = "output";
    config.physical_joint.motor.encoder.counts_per_output_revolution =
        kCountsPerRevolution;
    config.physical_joint.motor.encoder.gear_ratio = 101.0;
    config.motor_position_limits = {-1.2, 1.2, false};
    config.coordinate_transform = {1.0, 0.0};
    return config;
}

std::vector<robot::ti5::JointConfig> rightArmConfigs()
{
    const std::array<std::string, 7> names{
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow_yaw",
        "right_wrist_pitch",
        "right_wrist_yaw",
        "right_wrist_roll"};

    std::vector<robot::ti5::JointConfig> result;
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        result.push_back(jointConfig(
            names[index],
            "right_arm",
            static_cast<std::uint16_t>(16 + index)));
    }
    // Arm 必须按语义名称组装，而不是依赖 YAML 或调用方给出的顺序。
    std::reverse(result.begin(), result.end());
    return result;
}

std::vector<robot::ti5::JointConfig> leftArmConfigs()
{
    const std::array<std::string, 7> names{
        "left_shoulder_pitch",
        "left_shoulder_roll",
        "left_shoulder_yaw",
        "left_elbow_yaw",
        "left_wrist_pitch",
        "left_wrist_yaw",
        "left_wrist_roll"};

    std::vector<robot::ti5::JointConfig> result;
    for (std::size_t index = 0; index < names.size(); ++index)
    {
        result.push_back(jointConfig(
            names[index],
            "left_arm",
            static_cast<std::uint16_t>(23 + index)));
    }
    std::reverse(result.begin(), result.end());
    return result;
}

std::size_t countCommand(const std::vector<robot::can::CanFrame> &frames,
                         const std::uint8_t command)
{
    return static_cast<std::size_t>(std::count_if(
        frames.begin(),
        frames.end(),
        [command](const robot::can::CanFrame &frame)
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
        SimulatedArm simulation(*transport_pointer, 16);
        auto bus = std::make_unique<robot::ti5::CanBus>(
            std::move(transport));

        robot::ti5::ArmOptions options;
        options.control_period = std::chrono::milliseconds{0};
        options.inter_frame_gap = std::chrono::microseconds{0};
        options.position_control_start_cycles = 3;
        options.maximum_stale_cycles = 2;
        options.maximum_feedback_age = std::chrono::seconds{1};
        options.stop_inter_joint_gap = std::chrono::milliseconds{0};
        options.stop_settle_time = std::chrono::milliseconds{0};

        robot::ti5::Arm arm(
            robot::ti5::ArmSide::Right,
            std::move(bus),
            rightArmConfigs(),
            options);

        expect(arm.logicalBusName() == "right_arm" &&
                   arm.jointNames().front() == "right_shoulder_pitch" &&
                   arm.jointNames().back() == "right_wrist_roll" &&
                   arm.joint(0).nodeId() == 16 &&
                   arm.joint(6).nodeId() == 22,
               "Arm did not assemble joints by semantic name");

        arm.prepare();
        expect(arm.controlState() == robot::ti5::ArmControlState::Stopped,
               "prepared mode-0 Arm was not recorded as stopped");
        expect(!arm.hasSentPositionCommand(),
               "prepare must reset the position-command evidence flag");

        arm.startPositionControlAtCurrentPosition();
        expect(arm.controlState() ==
                   robot::ti5::ArmControlState::PositionControlActive,
               "Arm did not establish verified position control");
        expect(arm.hasSentPositionCommand(),
               "Arm did not record successful 0x44 transmission");

        robot::ti5::ArmController controller(arm);
        const auto position_frames_before_controller_start =
            countCommand(transport_pointer->sent_frames, 0x44);
        controller.start();
        expect(controller.state() ==
                   robot::ti5::ArmController::ControlState::Running &&
                   controller.currentState() &&
                   controller.targetReached(1e-9) &&
                   countCommand(transport_pointer->sent_frames, 0x44) ==
                       position_frames_before_controller_start,
               "ArmController did not start from feedback without sending");

        const auto state = arm.readState();
        expect(state.all_positions_available &&
                   state.all_csp_feedback_fresh,
               "Arm did not aggregate fresh seven-joint feedback");
        for (const auto &joint_state : state.joints)
        {
            expect(joint_state.position_rad &&
                       std::abs(*joint_state.position_rad) < 1e-12 &&
                       joint_state.velocity_rad_s &&
                       std::abs(*joint_state.velocity_rad_s -
                                2.0 * kPi / 100.0) < 1e-12 &&
                       joint_state.current_amps &&
                       std::abs(*joint_state.current_amps - 1.0) < 1e-12 &&
                       joint_state.run_mode && *joint_state.run_mode == 8 &&
                       joint_state.fault_bits && *joint_state.fault_bits == 0,
                   "Arm joint feedback conversion or status aggregation failed");
        }

        robot::ti5::Arm::JointValues invalid_target{};
        invalid_target.back() = 1.1;
        const auto position_frames_before_rejection =
            countCommand(transport_pointer->sent_frames, 0x44);
        expectThrow<std::out_of_range>(
            [&controller, &invalid_target]()
            {
                controller.setTarget(invalid_target);
            },
            "ArmController accepted a target outside driver limits");
        expect(countCommand(transport_pointer->sent_frames, 0x44) ==
                   position_frames_before_rejection,
               "ArmController sent before rejecting one invalid joint");
        expect(controller.state() ==
                   robot::ti5::ArmController::ControlState::Running,
               "target validation error incorrectly failed ArmController");

        robot::ti5::Arm::JointValues valid_target{};
        valid_target.fill(0.1);
        controller.setTarget(valid_target);
        expect(countCommand(transport_pointer->sent_frames, 0x44) ==
                   position_frames_before_rejection,
               "ArmController setTarget sent before update");
        controller.update();
        expect(countCommand(transport_pointer->sent_frames, 0x44) ==
                   position_frames_before_rejection + 7,
               "ArmController update did not send seven position frames");
        expect(controller.targetReached(5e-5),
               "ArmController did not retain updated feedback");
        controller.holdCurrentPosition();
        expect(controller.targetReached(5e-5),
               "ArmController hold did not use current feedback");

        const auto stop_start = transport_pointer->sent_frames.size();
        controller.stopAndConfirm();
        expect(arm.controlState() == robot::ti5::ArmControlState::Stopped,
               "ArmController did not confirm seven-joint mode 0 after STOP");
        expect(controller.state() ==
                   robot::ti5::ArmController::ControlState::Idle,
               "ArmController did not return to Idle after confirmed STOP");
        expect(transport_pointer->sent_frames.size() >= stop_start + 7,
               "Arm did not send seven STOP requests");
        for (std::size_t index = 0; index < 7; ++index)
        {
            const auto &frame = transport_pointer->sent_frames[
                stop_start + index];
            expect(frame.data_length == 1 && frame.data[0] == 0x02 &&
                       frame.id == static_cast<std::uint16_t>(22 - index),
                   "Arm STOP order was not wrist-to-shoulder");
        }
        arm.prepare();
        expect(!arm.hasSentPositionCommand(),
               "a new prepare cycle did not clear old 0x44 evidence");

        auto missing = rightArmConfigs();
        missing.pop_back();
        expectThrow<std::invalid_argument>(
            [&missing]()
            {
                auto missing_transport = std::make_unique<FakeTransport>();
                auto missing_bus = std::make_unique<robot::ti5::CanBus>(
                    std::move(missing_transport));
                robot::ti5::Arm invalid_arm(
                    robot::ti5::ArmSide::Right,
                    std::move(missing_bus),
                    missing);
            },
            "Arm accepted an incomplete semantic joint set");

        // 左臂使用另一组语义名称和节点号；显式 STOP 也必须能在尚未
        // prepare 时使用，避免准备失败后反而失去停止通道。
        auto left_transport = std::make_unique<FakeTransport>();
        auto *left_transport_pointer = left_transport.get();
        SimulatedArm left_simulation(*left_transport_pointer, 23);
        auto left_bus = std::make_unique<robot::ti5::CanBus>(
            std::move(left_transport));
        robot::ti5::Arm left_arm(
            robot::ti5::ArmSide::Left,
            std::move(left_bus),
            leftArmConfigs(),
            options);
        expect(left_arm.jointNames().front() == "left_shoulder_pitch" &&
                   left_arm.joint(0).nodeId() == 23 &&
                   left_arm.joint(6).nodeId() == 29,
               "left Arm semantic assembly mismatch");
        left_arm.requestStopModeAndConfirm();
        expect(left_arm.controlState() ==
                   robot::ti5::ArmControlState::Stopped,
               "unprepared Arm explicit STOP did not confirm mode 0");

        std::cout << "TI5 Arm tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 Arm test failed: " << error.what() << '\n';
        return 1;
    }
}
