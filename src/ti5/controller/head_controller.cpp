#include "ti5/controller/head_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::ti5
{

HeadController::HeadController(Head &head)
    : head_(head)
{
}

Head::JointValues HeadController::requireControllablePositions(
    const HeadState &state)
{
    if (!state.all_positions_available ||
        !state.all_csp_feedback_fresh)
    {
        throw std::runtime_error(
            "TI5 HeadController requires complete fresh CSP feedback");
    }

    Head::JointValues positions{};
    for (std::size_t index = 0; index < Head::kJointCount; ++index)
    {
        const auto &joint_state = state.joints[index];
        if (!joint_state.position_rad ||
            !joint_state.run_mode || *joint_state.run_mode != 8 ||
            !joint_state.fault_bits || *joint_state.fault_bits != 0)
        {
            throw std::runtime_error(
                "TI5 HeadController requires mode 8, fault 0 and position feedback");
        }
        positions[index] = *joint_state.position_rad;
    }
    return positions;
}

void HeadController::start()
{
    if (state_ != ControlState::Idle)
    {
        throw std::logic_error(
            "TI5 HeadController can only start from Idle state");
    }
    if (head_.controlState() != HeadControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 HeadController requires active Head position control");
    }

    try
    {
        const auto snapshot = head_.readState();
        const auto positions = requireControllablePositions(snapshot);
        head_.validatePositions(positions);
        current_state_ = snapshot;
        target_positions_ = positions;
        state_ = ControlState::Running;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void HeadController::setTarget(
    const Head::JointValues &target_positions)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 HeadController target requires Running state");
    }
    head_.validatePositions(target_positions);
    target_positions_ = target_positions;
}

void HeadController::holdCurrentPosition()
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 HeadController hold requires Running state");
    }

    try
    {
        const auto snapshot = head_.readState();
        const auto positions = requireControllablePositions(snapshot);
        head_.validatePositions(positions);
        current_state_ = snapshot;
        target_positions_ = positions;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void HeadController::stopAndConfirm()
{
    if (state_ != ControlState::Running &&
        state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 HeadController STOP requires Running or Failed state");
    }

    try
    {
        head_.requestStopModeAndConfirm();
        current_state_ = head_.readState();
        state_ = ControlState::Idle;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void HeadController::reset()
{
    if (state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 HeadController can only reset from Failed state");
    }
    current_state_.reset();
    state_ = ControlState::Idle;
}

void HeadController::update()
{
    if (state_ == ControlState::Failed)
    {
        return;
    }

    try
    {
        current_state_ = head_.readState();
        if (state_ != ControlState::Running)
        {
            return;
        }
        head_.commandPositionsCsp(target_positions_);
        current_state_ = head_.readState();
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

HeadController::ControlState HeadController::state() const noexcept
{
    return state_;
}

const std::optional<HeadState> &HeadController::currentState() const noexcept
{
    return current_state_;
}

const Head::JointValues &HeadController::targetPositions() const noexcept
{
    return target_positions_;
}

bool HeadController::targetReached(
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

    for (std::size_t index = 0; index < Head::kJointCount; ++index)
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
