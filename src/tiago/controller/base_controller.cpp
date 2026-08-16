#include "tiago/controller/base_controller.hpp"

#include <stdexcept>

namespace robot::tiago
{
    BaseController::BaseController(Base &base)
        : base_(base)
    {
    }

    void BaseController::start()
    {
        if (state_ != ControlState::Idle)
        {
            throw std::logic_error("BaseController can only start from Idle state");
        }

        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;
        state_ = ControlState::Running;
    }

    void BaseController::setTarget(double linear_velocity, double angular_velocity)
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("Target can only be updated while controller is Running");
        }

        // 先校验新目标，再修改内部状态，保证非法命令不会覆盖旧目标。
        base_.validateVelocityCommand(linear_velocity, angular_velocity);
        linear_velocity_ = linear_velocity;
        angular_velocity_ = angular_velocity;
    }

    void BaseController::stop()
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("BaseController can only stop from Running state");
        }

        try
        {
            base_.stop();
            linear_velocity_ = 0.0;
            angular_velocity_ = 0.0;
            state_ = ControlState::Idle;
        }
        catch (...)
        {
            // 停止命令失败时不能假设底盘已经停下，进入故障状态等待人工处理。
            state_ = ControlState::Failed;
            throw;
        }
    }

    void BaseController::reset()
    {
        if (state_ != ControlState::Failed)
        {
            throw std::logic_error("BaseController can only reset from Failed state");
        }

        linear_velocity_ = 0.0;
        angular_velocity_ = 0.0;
        state_ = ControlState::Idle;
    }

    void BaseController::update()
    {
        if (state_ != ControlState::Running)
        {
            return;
        }

        try
        {
            // 每次只发送当前目标，因此新目标会在下一次周期刷新时生效。
            base_.commandVelocity(linear_velocity_, angular_velocity_);
        }
        catch (...)
        {
            state_ = ControlState::Failed;
            throw;
        }
    }

    BaseController::ControlState BaseController::state() const noexcept
    {
        return state_;
    }

    double BaseController::linearVelocityTarget() const noexcept
    {
        return linear_velocity_;
    }

    double BaseController::angularVelocityTarget() const noexcept
    {
        return angular_velocity_;
    }
}
