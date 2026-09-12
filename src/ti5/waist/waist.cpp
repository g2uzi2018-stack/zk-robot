#include "ti5/waist/waist.hpp"

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

void validateOptions(const WaistOptions &options)
{
    if (options.control_period.count() < 0 ||
        options.inter_frame_gap.count() < 0 ||
        options.position_control_start_cycles == 0 ||
        options.maximum_stale_cycles == 0 ||
        options.maximum_feedback_age.count() <= 0 ||
        !std::isfinite(options.start_position_tolerance_rad) ||
        options.start_position_tolerance_rad <= 0.0)
    {
        throw std::invalid_argument("TI5 Waist options are invalid");
    }
}

} // namespace

Waist::Waist(std::unique_ptr<CanBus> bus,
           const std::vector<JointConfig> &available_joint_configs,
           WaistOptions options)
    : options_(options),
      bus_(std::move(bus))
{
    if (!bus_)
    {
        throw std::invalid_argument("TI5 Waist requires a CAN bus");
    }
    validateOptions(options_);

    std::map<std::string, const JointConfig *> by_name;
    for (const auto &config : available_joint_configs)
    {
        if (std::find(joint_names_.begin(),
                      joint_names_.end(),
                      config.physical_joint.name) == joint_names_.end())
        {
            continue;
        }
        if (!by_name.emplace(config.physical_joint.name, &config).second)
        {
            throw std::invalid_argument(
                "TI5 Waist contains duplicate semantic joint name: " +
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
                "TI5 Waist is missing semantic joint: " +
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
                "TI5 Waist contains duplicate CAN node ID");
        }
        if (found->second->physical_joint.motor.node_id != index + 1)
        {
            throw std::invalid_argument("TI5 Waist semantic joint/node mapping mismatch");
        }
        configs_[index] = *found->second;
        joints_[index] = std::make_unique<Joint>(
            configs_[index], *bus_);
    }
}

const std::string &Waist::logicalBusName() const noexcept
{
    return logical_bus_name_;
}

const Waist::JointNames &Waist::jointNames() const noexcept
{
    return joint_names_;
}

WaistControlState Waist::controlState() const noexcept
{
    return control_state_;
}

const Joint &Waist::joint(const std::size_t index) const
{
    return *joints_.at(index);
}

void Waist::requireHealthyBus(const char *operation) const
{
    const auto health = bus_->health();
    if (health.state != CanBusState::Healthy ||
        health.send_failure_latched)
    {
        throw std::runtime_error(
            std::string{"TI5 Waist CAN bus is not healthy before "} +
            operation);
    }
}

Waist::JointValues Waist::queryCurrentPositions()
{
    JointValues positions{};
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        const auto position = joints_[index]->queryPosition();
        if (!position)
        {
            throw std::runtime_error(
                joint_names_[index] + " position query failed");
        }
        positions[index] = *position;
    }
    return positions;
}

std::array<DriverStatus, Waist::kJointCount> Waist::queryDriverStatuses()
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

void Waist::requireUniformStartableModes(
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
            "TI5 Waist has mixed mode 0/mode 8 state");
    }
}

void Waist::prepare()
{
    if (control_state_ == WaistControlState::StartingPositionControl ||
        control_state_ == WaistControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 Waist cannot prepare during active control");
    }

    control_state_ = WaistControlState::Unprepared;
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
                             ? WaistControlState::Stopped
                             : WaistControlState::Prepared;
    }
    catch (...)
    {
        control_state_ = WaistControlState::Failed;
        throw;
    }
}

WaistState Waist::readState()
{
    bus_->collectPendingFeedback();
    WaistState result;
    result.bus_health = bus_->health();
    result.all_positions_available = true;
    result.all_csp_feedback_fresh = true;
    const auto now = std::chrono::steady_clock::now();

    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        auto &output = result.joints[index];
        const auto motor_state = bus_->latestState(joints_[index]->nodeId());
        if (!motor_state)
        {
            result.all_positions_available = false;
            result.all_csp_feedback_fresh = false;
            continue;
        }

        const auto &motor_config = configs_[index].physical_joint.motor;
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
            output.velocity_rad_s = motor_velocity /
                                    configs_[index]
                                        .coordinate_transform.direction;
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
        output.csp_update_sequence = motor_state->csp_update_sequence;
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

Waist::JointPositions Waist::readPositions()
{
    const auto state = readState();
    JointPositions positions{};
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        positions[index] = state.joints[index].position_rad;
    }
    return positions;
}

void Waist::clearFault()
{
    if (control_state_ == WaistControlState::StartingPositionControl ||
        control_state_ == WaistControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 Waist cannot clear faults during active control");
    }

    try
    {
        requireHealthyBus("clearing faults");
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            joints_[index]->clearFault();
            if (index + 1 < kJointCount &&
                options_.inter_frame_gap.count() > 0)
            {
                std::this_thread::sleep_for(options_.inter_frame_gap);
            }
        }
    }
    catch (...)
    {
        control_state_ = WaistControlState::Failed;
        throw;
    }
}

void Waist::validatePositions(const JointValues &positions) const
{
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        joints_[index]->validatePositionCommand(positions[index]);
    }
}

void Waist::sendPositions(const JointValues &positions)
{
    for (std::size_t index = 0; index < kJointCount; ++index)
    {
        joints_[index]->commandPositionCsp(positions[index]);
        if (index + 1 < kJointCount &&
            options_.inter_frame_gap.count() > 0)
        {
            std::this_thread::sleep_for(options_.inter_frame_gap);
        }
    }
}

void Waist::startPositionControlAtCurrentPosition()
{
    if (control_state_ != WaistControlState::Prepared &&
        control_state_ != WaistControlState::Stopped)
    {
        throw std::logic_error(
            "TI5 Waist position control can only start after prepare");
    }

    try
    {
        const auto statuses = queryDriverStatuses();
        requireUniformStartableModes(statuses);
        requireHealthyBus("starting position control");
        const auto hold_positions = queryCurrentPositions();
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

        control_state_ = WaistControlState::StartingPositionControl;
        auto next_cycle = std::chrono::steady_clock::now();
        for (std::size_t cycle = 0;
             cycle < options_.position_control_start_cycles;
             ++cycle)
        {
            requireHealthyBus("position-control start cycle");
            if (cycle > 0)
            {
                const auto current_statuses = queryDriverStatuses();
                for (const auto &status : current_statuses)
                {
                    if (status.run_mode != kPositionMode || status.fault_bits != 0)
                    {
                        throw std::runtime_error("TI5 Waist lost mode 8/fault 0 during startup");
                    }
                }
            }
            sendPositions(hold_positions);
            next_cycle += options_.control_period;
            std::this_thread::sleep_until(next_cycle);
            const auto snapshot = readState();
            for (std::size_t index = 0; index < kJointCount; ++index)
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
                    std::abs(*joint_state.position_rad -
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

        sendPositions(hold_positions);
        std::this_thread::sleep_for(options_.control_period);
        const auto final_snapshot = readState();
        requireHealthyBus("finishing position-control start");
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (final_snapshot.joints[index].csp_update_sequence <=
                    previous_sequences[index] ||
                !final_snapshot.joints[index].position_rad ||
                std::abs(*final_snapshot.joints[index].position_rad -
                         hold_positions[index]) >
                    options_.start_position_tolerance_rad)
            {
                throw std::runtime_error(
                    joint_names_[index] +
                    " final position-control hold feedback is invalid");
            }
        }
        last_commanded_positions_ = hold_positions;
        control_state_ = WaistControlState::PositionControlActive;
    }
    catch (...)
    {
        control_state_ = WaistControlState::Failed;
        throw;
    }
}

void Waist::commandPositionsCsp(const JointValues &positions)
{
    if (control_state_ != WaistControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 Waist position command requires active position control");
    }
    validatePositions(positions);
    try
    {
        requireHealthyBus("position command");
        const auto snapshot = readState();
        if (!snapshot.all_positions_available ||
            !snapshot.all_csp_feedback_fresh)
        {
            throw std::runtime_error(
                "TI5 Waist rejects a new target because CSP feedback is stale");
        }
        const auto statuses = queryDriverStatuses();
        requireHealthyBus("checking current mode and fault");
        if (!readState().all_csp_feedback_fresh)
        {
            throw std::runtime_error("TI5 Waist feedback expired during status queries");
        }
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            const auto &state = statuses[index];
            if (state.run_mode != kPositionMode || state.fault_bits != 0)
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
        control_state_ = WaistControlState::Failed;
        throw;
    }
}

} // namespace robot::ti5
