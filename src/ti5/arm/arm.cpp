#include "ti5/arm/arm.hpp"

#include "ti5/can/encoder_conversion.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace robot::ti5
{
namespace
{

constexpr std::uint32_t kStopMode =
    static_cast<std::uint32_t>(DriverRunMode::Stop);
constexpr std::uint32_t kPositionMode =
    static_cast<std::uint32_t>(DriverRunMode::ProfilePosition);

void validateOptions(const ArmOptions &options)
{
    if (options.control_period.count() < 0 ||
        options.inter_frame_gap.count() < 0 ||
        options.position_control_start_cycles == 0 ||
        options.maximum_stale_cycles == 0 ||
        options.maximum_feedback_age.count() <= 0 ||
        options.stop_inter_joint_gap.count() < 0 ||
        options.stop_settle_time.count() < 0 ||
        !std::isfinite(options.start_position_tolerance_rad) ||
        options.start_position_tolerance_rad <= 0.0)
    {
        throw std::invalid_argument("TI5 Arm options are invalid");
    }
}

} // namespace

Arm::JointNames Arm::expectedJointNames(const ArmSide side)
{
    if (side == ArmSide::Left)
    {
        return {
            "left_shoulder_pitch",
            "left_shoulder_roll",
            "left_shoulder_yaw",
            "left_elbow_yaw",
            "left_wrist_pitch",
            "left_wrist_yaw",
            "left_wrist_roll"};
    }
    return {
        "right_shoulder_pitch",
        "right_shoulder_roll",
        "right_shoulder_yaw",
        "right_elbow_yaw",
        "right_wrist_pitch",
        "right_wrist_yaw",
        "right_wrist_roll"};
}

std::string Arm::expectedBusName(const ArmSide side)
{
    return side == ArmSide::Left ? "left_arm" : "right_arm";
}

Arm::Arm(const ArmSide side,
         std::unique_ptr<CanBus> bus,
         const std::vector<JointConfig> &available_joint_configs,
         ArmOptions options)
    : side_(side),
      logical_bus_name_(expectedBusName(side)),
      joint_names_(expectedJointNames(side)),
      options_(options),
      bus_(std::move(bus))
{
    if (!bus_)
    {
        throw std::invalid_argument("TI5 Arm requires a CAN bus");
    }
    validateOptions(options_);

    std::map<std::string, const JointConfig *> by_name;
    for (const auto &config : available_joint_configs)
    {
        const auto expected = std::find(
            joint_names_.begin(),
            joint_names_.end(),
            config.physical_joint.name);
        if (expected == joint_names_.end())
        {
            continue;
        }
        if (!by_name.emplace(
                 config.physical_joint.name,
                 &config)
                 .second)
        {
            throw std::invalid_argument(
                "TI5 Arm contains duplicate semantic joint name: " +
                config.physical_joint.name);
        }
    }

    std::set<std::uint16_t> node_ids;
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const auto found = by_name.find(joint_names_[index]);
        if (found == by_name.end())
        {
            throw std::invalid_argument(
                "TI5 Arm is missing semantic joint: " +
                joint_names_[index]);
        }
        if (found->second->physical_joint.bus != logical_bus_name_)
        {
            throw std::invalid_argument(
                joint_names_[index] +
                " belongs to the wrong logical CAN bus");
        }
        if (!node_ids.insert(
                 found->second->physical_joint.motor.node_id)
                 .second)
        {
            throw std::invalid_argument(
                "TI5 Arm contains duplicate CAN node ID");
        }

        configs_[index] = *found->second;
        joints_[index] = std::make_unique<Joint>(
            configs_[index],
            *bus_);
    }
}

ArmSide Arm::side() const noexcept
{
    return side_;
}

const std::string &Arm::logicalBusName() const noexcept
{
    return logical_bus_name_;
}

const Arm::JointNames &Arm::jointNames() const noexcept
{
    return joint_names_;
}

ArmControlState Arm::controlState() const noexcept
{
    return control_state_;
}

bool Arm::hasSentPositionCommand() const noexcept
{
    return position_command_sent_;
}

Joint &Arm::joint(const std::size_t index)
{
    return *joints_.at(index);
}

const Joint &Arm::joint(const std::size_t index) const
{
    return *joints_.at(index);
}

void Arm::requireHealthyBus(const char *operation) const
{
    const auto health = bus_->health();
    if (health.state != CanBusState::Healthy ||
        health.send_failure_latched)
    {
        throw std::runtime_error(
            std::string{"TI5 Arm CAN bus is not healthy before "} +
            operation);
    }
}

Arm::JointValues Arm::queryCurrentPositions()
{
    JointValues positions{};
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const auto position = joints_[index]->queryPosition();
        if (!position)
        {
            throw std::runtime_error(
                joint_names_[index] +
                " position query failed");
        }
        positions[index] = *position;
    }
    return positions;
}

std::array<DriverStatus, Arm::kJointCount> Arm::queryDriverStatuses()
{
    std::array<DriverStatus, kJointCount> statuses{};
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const auto status = joints_[index]->queryDriverStatus();
        if (!status)
        {
            throw std::runtime_error(
                joint_names_[index] +
                " run-mode or fault query failed");
        }
        statuses[index] = *status;
    }
    return statuses;
}

void Arm::requireUniformStartableModes(
    const std::array<DriverStatus, kJointCount> &statuses) const
{
    bool all_stopped = true;
    bool all_position = true;
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const auto &status = statuses[index];
        if (status.fault_bits != 0)
        {
            std::ostringstream message;
            message << joint_names_[index]
                    << " has fault bits 0x" << std::hex
                    << status.fault_bits;
            throw std::runtime_error(message.str());
        }
        if (status.run_mode != kStopMode &&
            status.run_mode != kPositionMode)
        {
            throw std::runtime_error(
                joint_names_[index] +
                " is not in mode 0 or mode 8");
        }
        all_stopped = all_stopped && status.run_mode == kStopMode;
        all_position = all_position && status.run_mode == kPositionMode;
    }
    if (!all_stopped && !all_position)
    {
        throw std::runtime_error(
            "TI5 Arm has mixed mode 0/mode 8 state");
    }
}

void Arm::prepare()
{
    if (control_state_ == ArmControlState::StartingPositionControl ||
        control_state_ == ArmControlState::PositionControlActive ||
        control_state_ == ArmControlState::RequestingStop)
    {
        throw std::logic_error(
            "TI5 Arm cannot prepare during active control");
    }

    control_state_ = ArmControlState::Unprepared;
    position_command_sent_ = false;
    try
    {
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (!joints_[index]->refreshDriverPositionLimits())
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " driver position-limit query failed");
            }
        }

        static_cast<void>(queryCurrentPositions());
        const auto statuses = queryDriverStatuses();
        requireUniformStartableModes(statuses);
        requireHealthyBus("prepare");

        control_state_ = statuses.front().run_mode == kStopMode
                             ? ArmControlState::Stopped
                             : ArmControlState::Prepared;
    }
    catch (...)
    {
        control_state_ = ArmControlState::Failed;
        throw;
    }
}

ArmState Arm::readState()
{
    bus_->collectPendingFeedback();

    ArmState result;
    result.bus_health = bus_->health();
    result.all_positions_available = true;
    result.all_csp_feedback_fresh = true;
    const auto now = std::chrono::steady_clock::now();

    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        auto &output = result.joints[index];
        const auto motor_state = bus_->latestState(
            joints_[index]->nodeId());
        if (!motor_state)
        {
            result.all_positions_available = false;
            result.all_csp_feedback_fresh = false;
            continue;
        }

        const auto &motor_config =
            configs_[index].physical_joint.motor;
        if (motor_state->position_counts)
        {
            const double motor_position = positionCountsToRadians(
                motor_state->position_counts->value,
                motor_config.encoder.counts_per_output_revolution);
            output.position_rad =
                joints_[index]->motorToJointPosition(motor_position);
        }
        else
        {
            result.all_positions_available = false;
        }

        if (motor_state->speed_raw)
        {
            const double motor_velocity =
                speedRawToOutputRadiansPerSecond(
                    motor_state->speed_raw->value,
                    motor_config.encoder.gear_ratio);
            output.velocity_rad_s =
                motor_velocity /
                configs_[index].coordinate_transform.direction;
        }
        if (motor_state->current_milliamps)
        {
            output.current_amps =
                static_cast<double>(
                    motor_state->current_milliamps->value) /
                1000.0;
        }
        if (motor_state->run_mode)
        {
            output.run_mode = motor_state->run_mode->value;
        }
        if (motor_state->fault_bits)
        {
            output.fault_bits = motor_state->fault_bits->value;
        }
        output.csp_update_sequence =
            motor_state->csp_update_sequence;
        output.last_csp_feedback_timestamp =
            motor_state->last_csp_feedback_timestamp;

        if (!output.last_csp_feedback_timestamp ||
            now - *output.last_csp_feedback_timestamp >
                options_.maximum_feedback_age)
        {
            result.all_csp_feedback_fresh = false;
        }
    }
    return result;
}

void Arm::validatePositions(const JointValues &positions) const
{
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        joints_[index]->validatePositionCommand(positions[index]);
    }
}

void Arm::sendPositions(const JointValues &positions)
{
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        joints_[index]->commandPositionCsp(positions[index]);
        position_command_sent_ = true;
        if (index + 1 < kJointCount &&
            options_.inter_frame_gap.count() > 0)
        {
            std::this_thread::sleep_for(options_.inter_frame_gap);
        }
    }
}

void Arm::startPositionControlAtCurrentPosition()
{
    if (control_state_ != ArmControlState::Prepared &&
        control_state_ != ArmControlState::Stopped)
    {
        throw std::logic_error(
            "TI5 Arm position control can only start after prepare");
    }

    const auto statuses = queryDriverStatuses();
    requireUniformStartableModes(statuses);
    requireHealthyBus("starting position control");

    const JointValues hold_positions = queryCurrentPositions();
    // 先检查完 7 轴，保证任何一个目标不合法时一帧都不发送。
    validatePositions(hold_positions);

    const auto baseline = readState();
    std::array<std::uint64_t, kJointCount> previous_sequences{};
    std::array<std::size_t, kJointCount> stale_cycles{};
    std::array<std::size_t, kJointCount> fresh_cycles{};
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        previous_sequences[index] =
            baseline.joints[index].csp_update_sequence;
    }

    control_state_ = ArmControlState::StartingPositionControl;
    try
    {
        auto next_cycle = std::chrono::steady_clock::now();
        for (std::size_t cycle = 0;
             cycle < options_.position_control_start_cycles;
             ++cycle)
        {
            sendPositions(hold_positions);
            next_cycle += options_.control_period;
            std::this_thread::sleep_until(next_cycle);

            const auto snapshot = readState();
            for (std::size_t index = 0;
                 index < kJointCount;
                 ++index)
            {
                const auto &joint_state = snapshot.joints[index];
                if (joint_state.csp_update_sequence <=
                    previous_sequences[index])
                {
                    ++stale_cycles[index];
                    if (stale_cycles[index] >=
                        options_.maximum_stale_cycles)
                    {
                        throw std::runtime_error(
                            joint_names_[index] +
                            " CSP feedback became stale while starting position control");
                    }
                    continue;
                }

                previous_sequences[index] =
                    joint_state.csp_update_sequence;
                stale_cycles[index] = 0;
                ++fresh_cycles[index];
                if (!joint_state.position_rad ||
                    std::abs(
                        *joint_state.position_rad -
                        hold_positions[index]) >
                        options_.start_position_tolerance_rad)
                {
                    throw std::runtime_error(
                        joint_names_[index] +
                        " moved outside the position-control start envelope");
                }
            }
        }

        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (fresh_cycles[index] == 0)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " produced no fresh CSP feedback while starting position control");
            }
        }

        const auto final_statuses = queryDriverStatuses();
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (final_statuses[index].run_mode != kPositionMode ||
                final_statuses[index].fault_bits != 0)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " did not enter mode 8 with fault 0");
            }
        }

        // 状态查询期间可能没有 CSP 回传；最后再发一批当前位置目标，
        // 保证返回给 Controller 时每一轴都有新鲜反馈。
        sendPositions(hold_positions);
        std::this_thread::sleep_for(options_.control_period);
        const auto final_snapshot = readState();
        requireHealthyBus("finishing position-control start");
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (final_snapshot.joints[index].csp_update_sequence <=
                    previous_sequences[index] ||
                !final_snapshot.joints[index].position_rad ||
                std::abs(
                    *final_snapshot.joints[index].position_rad -
                    hold_positions[index]) >
                    options_.start_position_tolerance_rad)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " final position-control hold feedback is invalid");
            }
        }

        last_commanded_positions_ = hold_positions;
        control_state_ = ArmControlState::PositionControlActive;
    }
    catch (...)
    {
        control_state_ = ArmControlState::Failed;
        throw;
    }
}

void Arm::commandPositionsCsp(const JointValues &positions)
{
    if (control_state_ != ArmControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 Arm position command requires active position control");
    }

    // 完整预检先于任何发送。
    validatePositions(positions);
    const auto snapshot = readState();
    try
    {
        requireHealthyBus("position command");
        if (!snapshot.all_positions_available ||
            !snapshot.all_csp_feedback_fresh)
        {
            throw std::runtime_error(
                "TI5 Arm rejects a new target because CSP feedback is stale");
        }
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            const auto &state = snapshot.joints[index];
            if (!state.run_mode || *state.run_mode != kPositionMode ||
                !state.fault_bits || *state.fault_bits != 0)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " is not in verified mode 8/fault 0 state");
            }
        }

        sendPositions(positions);
        last_commanded_positions_ = positions;
    }
    catch (...)
    {
        control_state_ = ArmControlState::Failed;
        throw;
    }
}

void Arm::requestStopModeAndConfirm()
{
    // STOP 是显式的恢复请求。即使 prepare 或位置控制建立失败，调用方
    // 仍应能在确认机械上已托住手臂后请求 mode=0；不能让 Failed 状态
    // 反过来封死停止通道。Arm 本身不支持并发调用。
    if (control_state_ == ArmControlState::StartingPositionControl ||
        control_state_ == ArmControlState::RequestingStop)
    {
        throw std::logic_error(
            "TI5 Arm cannot request confirmed STOP from its current state");
    }

    control_state_ = ArmControlState::RequestingStop;
    try
    {
        for (std::size_t reverse = kJointCount;
             reverse > 0;
             --reverse)
        {
            joints_[reverse - 1]->requestStopMode();
            if (reverse > 1 &&
                options_.stop_inter_joint_gap.count() > 0)
            {
                std::this_thread::sleep_for(
                    options_.stop_inter_joint_gap);
            }
        }
        std::this_thread::sleep_for(options_.stop_settle_time);

        const auto statuses = queryDriverStatuses();
        requireHealthyBus("confirming STOP mode");
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (statuses[index].run_mode != kStopMode ||
                statuses[index].fault_bits != 0)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " did not confirm mode 0/fault 0 after STOP request");
            }
        }
        control_state_ = ArmControlState::Stopped;
    }
    catch (...)
    {
        control_state_ = ArmControlState::Failed;
        throw;
    }
}

} // namespace robot::ti5
