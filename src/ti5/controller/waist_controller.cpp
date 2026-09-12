#include "ti5/controller/waist_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::ti5
{

WaistController::WaistController(Waist &waist)
    : waist_(waist)
{
}

Waist::JointValues WaistController::requireControllablePositions(
    const WaistState &state)
{
    if (!state.all_positions_available ||
        !state.all_csp_feedback_fresh)
    {
        throw std::runtime_error(
            "TI5 WaistController requires complete fresh CSP feedback");
    }

    Waist::JointValues positions{};
    for (std::size_t index = 0; index < Waist::kJointCount; ++index)
    {
        const auto &joint_state = state.joints[index];
        if (!joint_state.position_rad ||
            !joint_state.run_mode || *joint_state.run_mode != 8 ||
            !joint_state.fault_bits || *joint_state.fault_bits != 0)
        {
            throw std::runtime_error(
                "TI5 WaistController requires mode 8, fault 0 and position feedback");
        }
        positions[index] = *joint_state.position_rad;
    }
    return positions;
}

void WaistController::start()
{
    startWithMode(ControlMode::FullWaistFold);
}

void WaistController::startWaistPitchControl()
{
    startWithMode(ControlMode::WaistPitchOnly);
}

void WaistController::startWithMode(const ControlMode mode)
{
    if (state_ != ControlState::Idle)
    {
        throw std::logic_error(
            "TI5 WaistController can only start from Idle state");
    }
    if (waist_.controlState() != WaistControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 WaistController requires active Waist position control");
    }

    try
    {
        const auto snapshot = waist_.readState();
        const auto positions = requireControllablePositions(snapshot);
        waist_.validatePositions(positions);
        current_state_ = snapshot;
        target_positions_ = positions;
        control_mode_ = mode;
        state_ = ControlState::Running;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void WaistController::setTarget(
    const Waist::JointValues &target_positions)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 WaistController target requires Running state");
    }
    if (control_mode_ == ControlMode::WaistPitchOnly)
    {
        throw std::logic_error(
            "TI5 WaistController is in waist-pitch-only mode; use setWaistPitchTarget");
    }
    waist_.validatePositions(target_positions);
    target_positions_ = target_positions;
}

void WaistController::setWaistPitchTarget(const double pitch_rad)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 WaistController waist-pitch target requires Running state");
    }
    if (control_mode_ != ControlMode::WaistPitchOnly)
    {
        throw std::logic_error(
            "TI5 WaistController is not in waist-pitch-only mode");
    }

    auto next_target = target_positions_;
    next_target[Waist::kWaistPitchIndex] = pitch_rad;
    waist_.validatePositions(next_target);
    target_positions_ = next_target;
}

void WaistController::holdCurrentPosition()
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 WaistController hold requires Running state");
    }

    try
    {
        const auto snapshot = waist_.readState();
        const auto positions = requireControllablePositions(snapshot);
        waist_.validatePositions(positions);
        current_state_ = snapshot;
        target_positions_ = positions;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void WaistController::reset()
{
    if (state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 WaistController can only reset from Failed state");
    }
    current_state_.reset();
    control_mode_ = ControlMode::FullWaistFold;
    state_ = ControlState::Idle;
}

void WaistController::update()
{
    if (state_ == ControlState::Failed)
    {
        return;
    }

    try
    {
        current_state_ = waist_.readState();
        if (state_ != ControlState::Running)
        {
            return;
        }
        waist_.commandPositionsCsp(target_positions_);
        current_state_ = waist_.readState();
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

WaistController::ControlState WaistController::state() const noexcept
{
    return state_;
}

const std::optional<WaistState> &WaistController::currentState() const noexcept
{
    return current_state_;
}

const Waist::JointValues &WaistController::targetPositions() const noexcept
{
    return target_positions_;
}

WaistController::ControlMode WaistController::controlMode() const noexcept
{
    return control_mode_;
}

bool WaistController::targetReached(
    const double position_tolerance_rad) const
{
    if (!std::isfinite(position_tolerance_rad) ||
        position_tolerance_rad <= 0.0)
    {
        throw std::invalid_argument(
            "Position tolerance must be positive and finite");
    }
    if (!current_state_ || !current_state_->all_positions_available)
    {
        return false;
    }

    for (std::size_t index = 0; index < Waist::kJointCount; ++index)
    {
        const auto &position = current_state_->joints[index].position_rad;
        if (!position ||
            std::abs(*position - target_positions_[index]) >
                position_tolerance_rad)
        {
            return false;
        }
    }
    return true;
}

} // namespace robot::ti5
