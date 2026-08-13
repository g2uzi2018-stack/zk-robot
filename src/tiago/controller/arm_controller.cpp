#include "tiago/controller/arm_controller.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::tiago
{
    ArmController::ArmController(Arm &arm)
        : arm_(arm)
    {
    }

    void ArmController::start(const Arm::JointValues &initial_positions, const Arm::JointValues &velocity_limits)
    {
        if (state_ != ControlState::Idle)
        {
            throw std::logic_error("ArmController can only start from Idle state");
        }

        // start() 只建立控制目标，不立即发送 CAN 命令。
        //
        // 先检查完整的 7 Joint 命令是否合法，
        // 保证 Controller 不会进入 Running 后才发现目标非法。
        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            arm_.joint(i).validateCommand(initial_positions[i], velocity_limits[i]);
        }

        target_positions_ = initial_positions;
        velocity_limits_ = velocity_limits;

        state_ = ControlState::Running;
    }

    void ArmController::setTarget(const Arm::JointValues &target_positions, const Arm::JointValues &velocity_limits)
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("Target can only be updated while controller is Running");
        }

        // 和 start() 一样，先检查完整目标。
        //
        // 如果任意 Joint 参数非法，
        // 当前正在执行的旧目标保持不变。
        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            arm_.joint(i).validateCommand(target_positions[i], velocity_limits[i]);
        }

        // latest target wins。
        //
        // 这里只更新 Controller 内部状态，
        // 真正的 CAN 下发统一发生在 update()。
        target_positions_ = target_positions;
        velocity_limits_ = velocity_limits;
    }

    void ArmController::stop()
    {
        if (state_ != ControlState::Running)
        {
            throw std::logic_error("ArmController can only stop from Running state");
        }

        try
        {
            arm_.stop();

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
            throw std::logic_error("ArmController can only reset from Failed state");
        }

        state_ = ControlState::Idle;
    }

    void ArmController::update()
    {
        // Failed 状态下不继续主动访问硬件。
        // 需要由上层处理错误并调用 reset()。
        if (state_ == ControlState::Failed)
        {
            return;
        }

        try
        {
            // 每个控制周期都收集并保存最新关节位置。
            current_positions_ = arm_.readPositions();

            // Idle 时只更新反馈，不发送位置控制命令。
            if (state_ != ControlState::Running)
            {
                return;
            }

            // Running 时持续刷新当前最新目标。
            //
            // 如果期间 setTarget() 修改了目标，
            // 下一次 update() 自然开始发送新的目标。
            arm_.commandPositions(target_positions_, velocity_limits_);
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

    const Arm::JointPositions &ArmController::currentPositions() const noexcept
    {
        return current_positions_;
    }

    const Arm::JointValues &ArmController::targetPositions() const noexcept
    {
        return target_positions_;
    }

    const Arm::JointValues &ArmController::velocityLimits() const noexcept
    {
        return velocity_limits_;
    }

    bool ArmController::targetReached(double position_tolerance) const
    {
        if (!std::isfinite(position_tolerance) ||
            position_tolerance <= 0.0)
        {
            throw std::invalid_argument("Position tolerance must be a positive finite value");
        }

        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            // 任意一个 Joint 没有反馈，
            // 都不能认为当前目标已经到达。
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
