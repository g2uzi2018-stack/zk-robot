#include "ti5/controller/hand_controller.hpp"

#include <stdexcept>

namespace robot::ti5
{

HandController::HandController(Hand &hand)
    : hand_(hand)
{
}

void HandController::start(
    const Hand::SpeedValues &holding_speeds)
{
    if (state_ != ControlState::Idle)
    {
        throw std::logic_error(
            "TI5 HandController can only start from Idle state");
    }
    if (!hand_.controlAllowed())
    {
        throw std::logic_error(
            "TI5 HandController control is disabled by configuration");
    }

    try
    {
        const auto snapshot = hand_.readState();
        if (!snapshot)
        {
            throw std::runtime_error(
                "TI5 HandController requires current hand status");
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

void HandController::setTarget(
    const Hand::PositionValues &target_positions,
    const Hand::SpeedValues &speeds)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 HandController target requires Running state");
    }
    target_positions_ = target_positions;
    speeds_ = speeds;
}

void HandController::pause()
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 HandController can only pause from Running state");
    }
    state_ = ControlState::Idle;
}

void HandController::reset()
{
    if (state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 HandController can only reset from Failed state");
    }
    current_state_.reset();
    state_ = ControlState::Idle;
}

void HandController::update()
{
    if (state_ == ControlState::Failed)
    {
        return;
    }

    try
    {
        current_state_ = hand_.readState();
        if (!current_state_)
        {
            throw std::runtime_error(
                "TI5 HandController did not receive hand status");
        }
        if (state_ == ControlState::Running)
        {
            hand_.commandPositionsRaw(target_positions_, speeds_);
        }
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

HandController::ControlState HandController::state() const noexcept
{
    return state_;
}

const std::optional<HandState> &
HandController::currentState() const noexcept
{
    return current_state_;
}

const Hand::PositionValues &
HandController::targetPositions() const noexcept
{
    return target_positions_;
}

const Hand::SpeedValues &HandController::speeds() const noexcept
{
    return speeds_;
}

bool HandController::targetReachedRaw(
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
