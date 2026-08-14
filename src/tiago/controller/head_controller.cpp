#include "tiago/controller/head_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::tiago
{
    HeadController::HeadController(Head &head)
        : head_(head)
    {
    }

    void HeadController::start(const Head::JointValues &initial_positions, const Head::JointValues &velocity_limits)
    {
        if (state_ != ControlState::Idle)
        {
            throw std::logic_error("HeadController can only start from Idle state");
        }

        // start() 只建立控制目标，不立即发送 CAN 命令。
        //
        // 先检查完整的两轴命令，
        // 防止进入 Running 后才发现目标非法。
        for (std::size_t i = 0; i < Head::kJointCount; ++i)
        {
            head_.joint(i).validateCommand(initial_positions[i], velocity_limits[i]);
        }

        target_positions_ = initial_positions;
        velocity_limits_ = velocity_limits;

        state_ = ControlState::Running;
    }

    void HeadController::setTarget(const Head::JointValues &target_positions, const Head::JointValues &velocity_limits)
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("Target can only be updated while controller is Running");
        }

        // 先检查完整目标。
        //
        // 如果任意 Joint 非法，
        // 当前正在执行的旧目标保持不变。
        for (std::size_t i = 0; i < Head::kJointCount; ++i)
        {
            head_.joint(i).validateCommand(target_positions[i], velocity_limits[i]);
        }

        // latest target wins。
        //
        // 这里只修改内部目标，
        // 真正发送发生在下一次 update()。
        target_positions_ = target_positions;
        velocity_limits_ = velocity_limits;
    }

    void HeadController::stop()
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("HeadController can only stop from Running state");
        }

        try
        {
            head_.stop();

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
            throw std::logic_error("HeadController can only reset from Failed state");
        }

        state_ = ControlState::Idle;
    }

    void HeadController::update()
    {
        // Failed 后不再主动访问硬件。
        if (state_ == ControlState::Failed)
        {
            return;
        }

        try
        {
            // 每个周期都读取最新位置。
            current_positions_ = head_.readPositions();

            // Idle 只更新反馈。
            if (state_ != ControlState::Running)
            {
                return;
            }

            // Running 持续刷新最新目标。
            head_.commandPositions(target_positions_, velocity_limits_);
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

    const Head::JointPositions &HeadController::currentPositions() const noexcept
    {
        return current_positions_;
    }

    const Head::JointValues &HeadController::targetPositions() const noexcept
    {
        return target_positions_;
    }

    const Head::JointValues &HeadController::velocityLimits() const noexcept
    {
        return velocity_limits_;
    }

    bool HeadController::targetReached(double position_tolerance) const
    {
        if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0)
        {
            throw std::invalid_argument("Position tolerance must be a positive finite value");
        }

        for (std::size_t i = 0; i < Head::kJointCount; ++i)
        {
            if (!current_positions_[i])
            {
                return false;
            }

            const double error = std::abs(*current_positions_[i] - target_positions_[i]);

            if (error > position_tolerance)
            {
                return false;
            }
        }

        return true;
    }
}
