#include "tiago/executor/robot_control_executor.hpp"

#include <stdexcept>

namespace robot::tiago
{

    RobotControlExecutor::RobotControlExecutor(
        ArmController &left_arm,
        ArmController &right_arm,

        GripperController &left_gripper,
        GripperController &right_gripper,

        HeadController &head,
        TorsoController &torso,

        BaseController &base,

        Config config)
        : left_arm_(left_arm),
          right_arm_(right_arm),

          left_gripper_(left_gripper),
          right_gripper_(right_gripper),

          head_(head),
          torso_(torso),

          base_(base),

          config_(config)
    {
    }

    RobotControlExecutor::~RobotControlExecutor()
    {
        shutdown();
    }

    void RobotControlExecutor::start()
    {
        if (state_ != State::Stopped)
        {
            throw std::logic_error(
                "Executor can only start from Stopped state");
        }

        shutdown_requested_ = false;

        state_ = State::Running;

        robot::common::logger()->info(
            "RobotControlExecutor started");

        control_thread_ =
            std::thread(
                &RobotControlExecutor::controlLoop,
                this);
    }

    void RobotControlExecutor::shutdown()
    {
        shutdown_requested_ = true;

        if (control_thread_.joinable())
        {
            control_thread_.join();
        }

        state_ = State::Stopped;

        robot::common::logger()->info(
            "RobotControlExecutor stopped");
    }

    RobotControlExecutor::State
    RobotControlExecutor::state() const noexcept
    {
        return state_;
    }

    void RobotControlExecutor::setLeftArmControlMode(
        ArmControlMode mode)
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_left_arm_mode_ = mode;
    }

    void RobotControlExecutor::setRightArmControlMode(
        ArmControlMode mode)
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_right_arm_mode_ = mode;
    }

    void RobotControlExecutor::submitLeftArmTrajectory(
        std::shared_ptr<const ArmTrajectory> trajectory)
    {
        if (!trajectory)
        {
            throw std::invalid_argument(
                "Left trajectory is null");
        }

        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_left_trajectory_ =
            std::move(trajectory);
    }

    void RobotControlExecutor::submitRightArmTrajectory(
        std::shared_ptr<const ArmTrajectory> trajectory)
    {
        if (!trajectory)
        {
            throw std::invalid_argument(
                "Right trajectory is null");
        }

        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_right_trajectory_ =
            std::move(trajectory);
    }

    void RobotControlExecutor::setLeftArmServoTarget(
        const Arm::JointValues &positions,
        const Arm::JointValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_left_servo_target_ =
            ArmTarget{
                positions,
                velocity_limits};
    }

    void RobotControlExecutor::setRightArmServoTarget(
        const Arm::JointValues &positions,
        const Arm::JointValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        pending_right_servo_target_ =
            ArmTarget{
                positions,
                velocity_limits};
    }

    void RobotControlExecutor::holdLeftArm()
    {
        left_arm_runtime_.state =
            ArmMotionState::Holding;

        robot::common::logger()->info(
            "Left arm hold requested");
    }

    void RobotControlExecutor::holdRightArm()
    {
        right_arm_runtime_.state =
            ArmMotionState::Holding;

        robot::common::logger()->info(
            "Right arm hold requested");
    }

    void RobotControlExecutor::stopLeftArm()
    {
        left_arm_.stop();

        left_arm_runtime_.state =
            ArmMotionState::Inactive;

        left_arm_runtime_.active_trajectory.reset();

        robot::common::logger()->info(
            "Left arm stopped");
    }

    void RobotControlExecutor::stopRightArm()
    {
        right_arm_.stop();

        right_arm_runtime_.state =
            ArmMotionState::Inactive;

        right_arm_runtime_.active_trajectory.reset();

        robot::common::logger()->info(
            "Right arm stopped");
    }

    void RobotControlExecutor::controlLoop()
    {
        auto next_cycle =
            Clock::now();

        while (!shutdown_requested_)
        {
            next_cycle +=
                config_.control_period;

            const auto start =
                Clock::now();

            try
            {
                runCycle(start);
            }
            catch (const std::exception &error)
            {
                enterFault(
                    error.what());
            }

            const auto end =
                Clock::now();

            checkDeadline(
                start,
                end);

            std::this_thread::sleep_until(
                next_cycle);
        }
    }

    void RobotControlExecutor::runCycle(
        TimePoint now)
    {
        processCommands();

        sampleTrajectory(now);

        updateControllers();

        publishState(now);
    }

    void RobotControlExecutor::processCommands()
    {
        std::lock_guard<std::mutex> lock(
            command_mutex_);

        if (pending_left_arm_mode_)
        {
            left_arm_runtime_.mode =
                *pending_left_arm_mode_;

            robot::common::logger()->info(
                "Left arm mode changed");

            pending_left_arm_mode_.reset();
        }

        if (pending_right_arm_mode_)
        {
            right_arm_runtime_.mode =
                *pending_right_arm_mode_;

            robot::common::logger()->info(
                "Right arm mode changed");

            pending_right_arm_mode_.reset();
        }

        if (pending_left_trajectory_)
        {
            if (left_arm_runtime_.mode !=
                ArmControlMode::Trajectory)
            {
                throw std::logic_error(
                    "Left arm is not in trajectory mode");
            }

            left_arm_runtime_.active_trajectory =
                pending_left_trajectory_;

            left_arm_runtime_.trajectory_start_time =
                Clock::now();

            left_arm_runtime_.state =
                ArmMotionState::Running;

            pending_left_trajectory_.reset();

            robot::common::logger()->info(
                "Left trajectory activated");
        }

        if (pending_right_trajectory_)
        {
            if (right_arm_runtime_.mode !=
                ArmControlMode::Trajectory)
            {
                throw std::logic_error(
                    "Right arm is not in trajectory mode");
            }

            right_arm_runtime_.active_trajectory =
                pending_right_trajectory_;

            right_arm_runtime_.trajectory_start_time =
                Clock::now();

            right_arm_runtime_.state =
                ArmMotionState::Running;

            pending_right_trajectory_.reset();

            robot::common::logger()->info(
                "Right trajectory activated");
        }

        if (pending_left_servo_target_)
        {
            if (left_arm_runtime_.mode !=
                ArmControlMode::Servo)
            {
                throw std::logic_error(
                    "Left arm is not in servo mode");
            }

            left_arm_runtime_.servo_target =
                pending_left_servo_target_;

            left_arm_runtime_.state =
                ArmMotionState::Running;

            pending_left_servo_target_.reset();
        }

        if (pending_right_servo_target_)
        {
            if (right_arm_runtime_.mode !=
                ArmControlMode::Servo)
            {
                throw std::logic_error(
                    "Right arm is not in servo mode");
            }

            right_arm_runtime_.servo_target =
                pending_right_servo_target_;

            right_arm_runtime_.state =
                ArmMotionState::Running;

            pending_right_servo_target_.reset();
        }
    }

    void RobotControlExecutor::sampleTrajectory(
        TimePoint now)
    {
        if (left_arm_runtime_.active_trajectory)
        {
            const auto elapsed =
                now -
                left_arm_runtime_.trajectory_start_time;

            const auto point =
                left_arm_runtime_
                    .active_trajectory
                    ->sample(elapsed);

            // TODO:
            // velocity_limit来源需要后续确定。

            Arm::JointValues velocity_limits{};
            velocity_limits.fill(0.2);

            left_arm_.setTarget(
                point.position,
                velocity_limits);
        }
    }

    void RobotControlExecutor::updateControllers()
    {
        left_arm_.update();

        right_arm_.update();

        left_gripper_.update();

        right_gripper_.update();

        head_.update();

        torso_.update();

        base_.update();
    }

    void RobotControlExecutor::publishState(
        TimePoint now)
    {
        RobotState state;

        state.timestamp = now;

        state.left_arm_positions =
            left_arm_.currentPositions();

        state.right_arm_positions =
            right_arm_.currentPositions();

        state.left_arm_state =
            left_arm_.state();

        state.right_arm_state =
            right_arm_.state();

        state.base_state =
            base_.state();

        std::lock_guard<std::mutex> lock(
            state_mutex_);

        robot_state_ =
            state;
    }

    RobotControlExecutor::RobotState
    RobotControlExecutor::latestState() const
    {
        std::lock_guard<std::mutex> lock(
            state_mutex_);

        return robot_state_;
    }

    RobotControlExecutor::CycleStatistics
    RobotControlExecutor::statistics() const
    {
        std::lock_guard<std::mutex> lock(
            statistics_mutex_);

        return statistics_;
    }

    std::string RobotControlExecutor::faultMessage() const
    {
        std::lock_guard<std::mutex> lock(
            fault_mutex_);

        return fault_message_;
    }

    void RobotControlExecutor::checkDeadline(
        TimePoint start,
        TimePoint end)
    {
        const auto execution =
            end - start;

        std::lock_guard<std::mutex> lock(
            statistics_mutex_);

        statistics_.cycle_count++;

        statistics_.last_execution_time =
            execution;

        if (execution >
            config_.control_period)
        {
            statistics_.deadline_miss_count++;

            robot::common::logger()->warn(
                "Executor deadline miss");
        }
    }

    void RobotControlExecutor::enterFault(
        const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(
                fault_mutex_);

            fault_message_ =
                reason;
        }

        state_ =
            State::Faulted;

        robot::common::logger()->error(
            "Executor fault: {}",
            reason);
    }

}