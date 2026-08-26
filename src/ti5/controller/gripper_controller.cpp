#include "ti5/controller/gripper_controller.hpp"

#include <algorithm>
#include <stdexcept>

namespace robot::ti5
{

GripperController::GripperController(Gripper &gripper)
    : gripper_(gripper)
{
}

void GripperController::start(
    const Gripper::SpeedValues &holding_speeds)
{
    if (state_ != ControlState::Idle)
    {
        throw std::logic_error(
            "TI5 GripperController can only start from Idle state");
    }
    if (!gripper_.controlAllowed())
    {
        throw std::logic_error(
            "TI5 GripperController control is disabled by configuration");
    }

    try
    {
        const auto snapshot = gripper_.readState();
        if (!snapshot)
        {
            throw std::runtime_error(
                "TI5 GripperController requires current hand status");
        }
        current_state_ = snapshot;
        target_positions_ = snapshot->positions_raw;
        speeds_ = holding_speeds;
        state_ = ControlState::Running;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void GripperController::setTarget(
    const Gripper::PositionValues &target_positions,
    const Gripper::SpeedValues &speeds)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 GripperController target requires Running state");
    }
    target_positions_ = target_positions;
    speeds_ = speeds;
}

void GripperController::pause()
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 GripperController can only pause from Running state");
    }
    state_ = ControlState::Idle;
}

void GripperController::reset()
{
    if (state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 GripperController can only reset from Failed state");
    }
    current_state_.reset();
    state_ = ControlState::Idle;
}

void GripperController::update()
{
    if (state_ == ControlState::Failed)
    {
        return;
    }

    try
    {
        current_state_ = gripper_.readState();
        if (!current_state_)
        {
            throw std::runtime_error(
                "TI5 GripperController did not receive hand status");
        }
        if (state_ == ControlState::Running)
        {
            gripper_.commandPositionsRaw(target_positions_, speeds_);
        }
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

GripperController::ControlState GripperController::state() const noexcept
{
    return state_;
}

const std::optional<GripperState> &
GripperController::currentState() const noexcept
{
    return current_state_;
}

const Gripper::PositionValues &
GripperController::targetPositions() const noexcept
{
    return target_positions_;
}

const Gripper::SpeedValues &GripperController::speeds() const noexcept
{
    return speeds_;
}

bool GripperController::targetReachedRaw(
    const std::uint16_t position_tolerance_raw) const
{
    if (!current_state_)
    {
        return false;
    }

    for (std::size_t index = 0;
         index < hand::kAoyiChannelCount;
         ++index)
    {
        const auto current = current_state_->positions_raw[index];
        const auto target = target_positions_[index];
        const auto error = current > target
                               ? static_cast<std::uint32_t>(current - target)
                               : static_cast<std::uint32_t>(target - current);
        if (error > position_tolerance_raw)
        {
            return false;
        }
    }
    return true;
}

} // namespace robot::ti5
