#include "ti5/controller/arm_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::ti5
{

ArmController::ArmController(Arm &arm)
    : arm_(arm)
{
}

Arm::JointValues ArmController::requireControllablePositions(
    const ArmState &state)
{
    if (!state.all_positions_available ||
        !state.all_csp_feedback_fresh)
    {
        throw std::runtime_error(
            "TI5 ArmController requires complete fresh CSP feedback");
    }

    Arm::JointValues positions{};
    for (std::size_t index = 0; index < Arm::kJointCount; ++index)
    {
        const auto &joint_state = state.joints[index];
        if (!joint_state.position_rad ||
            !joint_state.run_mode || *joint_state.run_mode != 8 ||
            !joint_state.fault_bits || *joint_state.fault_bits != 0)
        {
            throw std::runtime_error(
                "TI5 ArmController requires mode 8, fault 0 and position feedback");
        }
        positions[index] = *joint_state.position_rad;
    }
    return positions;
}

void ArmController::start()
{
    if (state_ != ControlState::Idle)
    {
        throw std::logic_error(
            "TI5 ArmController can only start from Idle state");
    }
    if (arm_.controlState() != ArmControlState::PositionControlActive)
    {
        throw std::logic_error(
            "TI5 ArmController requires active Arm position control");
    }

    try
    {
        const auto snapshot = arm_.readState();
        const auto positions = requireControllablePositions(snapshot);
        arm_.validatePositions(positions);
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

void ArmController::setTarget(
    const Arm::JointValues &target_positions)
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 ArmController target requires Running state");
    }

    arm_.validatePositions(target_positions);
    target_positions_ = target_positions;
}

void ArmController::holdCurrentPosition()
{
    if (state_ != ControlState::Running)
    {
        throw std::logic_error(
            "TI5 ArmController hold requires Running state");
    }

    try
    {
        const auto snapshot = arm_.readState();
        const auto positions = requireControllablePositions(snapshot);
        arm_.validatePositions(positions);
        current_state_ = snapshot;
        target_positions_ = positions;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void ArmController::stopAndConfirm()
{
    if (state_ != ControlState::Running &&
        state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 ArmController STOP requires Running or Failed state");
    }

    try
    {
        arm_.requestStopModeAndConfirm();
        current_state_ = arm_.readState();
        state_ = ControlState::Idle;
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

void ArmController::reset()
{
    if (state_ != ControlState::Failed)
    {
        throw std::logic_error(
            "TI5 ArmController can only reset from Failed state");
    }
    current_state_.reset();
    state_ = ControlState::Idle;
}

void ArmController::update()
{
    if (state_ == ControlState::Failed)
    {
        return;
    }

    try
    {
        current_state_ = arm_.readState();
        if (state_ != ControlState::Running)
        {
            return;
        }

        arm_.commandPositionsCsp(target_positions_);
        current_state_ = arm_.readState();
    }
    catch (...)
    {
        state_ = ControlState::Failed;
        throw;
    }
}

ArmController::ControlState ArmController::state() const noexcept
{
    return state_;
}

const std::optional<ArmState> &ArmController::currentState() const noexcept
{
    return current_state_;
}

const Arm::JointValues &ArmController::targetPositions() const noexcept
{
    return target_positions_;
}

bool ArmController::targetReached(
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

    for (std::size_t index = 0; index < Arm::kJointCount; ++index)
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
