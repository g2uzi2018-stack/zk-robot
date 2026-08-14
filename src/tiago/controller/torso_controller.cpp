#include "tiago/controller/torso_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::tiago
{
    TorsoController::TorsoController(Torso &torso)
        : torso_(torso)
    {
    }

    void TorsoController::start(double initial_position, double velocity_limit)
    {
        if (state_ != ControlState::Idle)
        {
            throw std::logic_error("TorsoController can only start from Idle state");
        }

        // 先验证，合法后才能进入Running。
        torso_.joint().validateCommand(initial_position, velocity_limit);

        target_position_ = initial_position;
        velocity_limit_ = velocity_limit;

        state_ = ControlState::Running;
    }

    void TorsoController::setTarget(double target_position, double velocity_limit)
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("Target can only be updated while controller is Running");
        }

        // 非法目标不得覆盖当前正在执行的旧目标。
        torso_.joint().validateCommand(target_position, velocity_limit);

        target_position_ = target_position;
        velocity_limit_ = velocity_limit;
    }

    void TorsoController::stop()
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("TorsoController can only stop from Running state");
        }

        try
        {
            torso_.stop();

            state_ = ControlState::Idle;
        }
        catch (...)
        {
            state_ = ControlState::Failed;
            throw;
        }
    }

    void TorsoController::reset()
    {
        if (state_ != ControlState::Failed)
        {
            throw std::logic_error("TorsoController can only reset from Failed state");
        }

        state_ = ControlState::Idle;
    }

    void TorsoController::update()
    {
        if (state_ == ControlState::Failed)
        {
            return;
        }

        try
        {
            current_position_ = torso_.readPosition();

            if (state_ != ControlState::Running)
            {
                return;
            }

            torso_.commandPosition(target_position_, velocity_limit_);
        }
        catch (...)
        {
            state_ = ControlState::Failed;
            throw;
        }
    }

    TorsoController::ControlState TorsoController::state() const noexcept
    {
        return state_;
    }

    const std::optional<double> &TorsoController::currentPosition() const noexcept
    {
        return current_position_;
    }

    double TorsoController::targetPosition() const noexcept
    {
        return target_position_;
    }

    double TorsoController::velocityLimit() const noexcept
    {
        return velocity_limit_;
    }

    bool TorsoController::targetReached(double position_tolerance) const
    {
        if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0)
        {
            throw std::invalid_argument("Position tolerance must be a positive finite value");
        }

        if (!current_position_)
        {
            return false;
        }

        return std::abs(*current_position_ - target_position_) <= position_tolerance;
    }
}
