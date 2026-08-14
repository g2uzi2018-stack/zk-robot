#include "tiago/controller/gripper_controller.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace robot::tiago
{
    GripperController::GripperController(Gripper &gripper)
        : gripper_(gripper)
    {
    }

    void GripperController::start(const Gripper::FingerValues &initial_positions, const Gripper::FingerValues &velocity_limits)
    {
        if (state_ != ControlState::Idle)
        {
            throw std::logic_error("GripperController can only start from Idle state");
        }

        // start() 只建立控制目标，不立即发送 CAN 命令。
        //
        // 先检查两个 finger 的完整命令，
        // 避免进入 Running 后才发现目标非法。
        for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
        {
            gripper_.finger(i).validateCommand(initial_positions[i], velocity_limits[i]);
        }

        target_positions_ = initial_positions;
        velocity_limits_ = velocity_limits;

        state_ = ControlState::Running;
    }

    void GripperController::setTarget(const Gripper::FingerValues &target_positions, const Gripper::FingerValues &velocity_limits)
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("Target can only be updated while controller is Running");
        }

        // 先检查完整的两个 finger 目标。
        //
        // 任意一个 finger 参数非法时，
        // 当前正在执行的旧目标保持不变。
        for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
        {
            gripper_.finger(i).validateCommand(target_positions[i], velocity_limits[i]);
        }

        // latest target wins。
        //
        // 这里只更新 Controller 内部目标，
        // 实际 CAN 下发统一发生在 update()。
        target_positions_ = target_positions;
        velocity_limits_ = velocity_limits;
    }

    void GripperController::stop()
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("GripperController can only stop from Running state");
        }

        try
        {
            gripper_.stop();

            state_ = ControlState::Idle;
        }
        catch (...)
        {
            state_ = ControlState::Failed;
            throw;
        }
    }

    void GripperController::reset()
    {
        if (state_ != ControlState::Failed)
        {
            throw std::logic_error("GripperController can only reset from Failed state");
        }

        state_ = ControlState::Idle;
    }

    void GripperController::update()
    {
        // Failed 状态下不继续主动访问硬件。
        if (state_ == ControlState::Failed)
        {
            return;
        }

        try
        {
            // 每个周期都读取并保存两个 finger 的最新位置。
            current_positions_ = gripper_.readPositions();

            // Idle 状态只更新反馈。
            if (state_ != ControlState::Running)
            {
                return;
            }

            // Running 状态持续刷新当前最新目标。
            gripper_.commandPositions(target_positions_, velocity_limits_);
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

    const Gripper::FingerPositions &GripperController::currentPositions() const noexcept
    {
        return current_positions_;
    }

    const Gripper::FingerValues &GripperController::targetPositions() const noexcept
    {
        return target_positions_;
    }

    const Gripper::FingerValues &GripperController::velocityLimits() const noexcept
    {
        return velocity_limits_;
    }

    bool GripperController::targetReached(double position_tolerance) const
    {
        if (!std::isfinite(position_tolerance) || position_tolerance <= 0.0)
        {
            throw std::invalid_argument("Position tolerance must be a positive finite value");
        }

        for (std::size_t i = 0; i < Gripper::kFingerCount; ++i)
        {
            // 任意一个 finger 没有反馈，
            // 都不能认为目标已经到达。
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
