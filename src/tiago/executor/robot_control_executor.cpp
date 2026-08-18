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
            throw std::logic_error("RobotControlExecutor can only start from Stopped");
        }

        shutdown_requested_ = false;
        state_ = State::Running;
        control_thread_ = std::thread(&RobotControlExecutor::controlLoop, this);
    }

    void RobotControlExecutor::shutdown()
    {
        shutdown_requested_ = true;

        if (control_thread_.joinable())
        {
            control_thread_.join();
        }

        state_ = State::Stopped;
    }

    RobotControlExecutor::State RobotControlExecutor::state() const noexcept
    {
        return state_;
    }

    void RobotControlExecutor::setLeftArmControlMode(ArmControlMode mode)
    {
        left_arm_runtime_.mode = mode;
    }

    void RobotControlExecutor::setRightArmControlMode(ArmControlMode mode)
    {
        right_arm_runtime_.mode = mode;
    }

    void RobotControlExecutor::submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory)
    {
        if (!trajectory)
        {
            throw std::invalid_argument("Left arm trajectory is null");
        }

        if (left_arm_runtime_.mode != ArmControlMode::Trajectory)
        {
            throw std::logic_error("Left arm is not in trajectory mode");
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_left_trajectory_ = std::move(trajectory);
    }

    void RobotControlExecutor::submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory)
    {
        if (!trajectory)
        {
            throw std::invalid_argument("Right arm trajectory is null");
        }

        if (right_arm_runtime_.mode != ArmControlMode::Trajectory)
        {
            throw std::logic_error("Right arm is not in trajectory mode");
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_right_trajectory_ = std::move(trajectory);
    }

    void RobotControlExecutor::setLeftArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits)
    {
        if (left_arm_runtime_.mode != ArmControlMode::Servo)
        {
            throw std::logic_error("Left arm is not in servo mode");
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_left_servo_target_ = ArmTarget{positions, velocity_limits};
    }

    void RobotControlExecutor::setRightArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits)
    {
        if (right_arm_runtime_.mode != ArmControlMode::Servo)
        {
            throw std::logic_error("Right arm is not in servo mode");
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        pending_right_servo_target_ = ArmTarget{positions, velocity_limits};
    }

    void RobotControlExecutor::holdLeftArm()
    {
        const auto current = left_arm_.currentPositions();
        ArmTarget target;

        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            if (!current[i])
            {
                throw std::runtime_error("Left arm has invalid feedback");
            }

            target.positions[i] = *current[i];
            target.velocity_limits[i] = 0.1;
        }

        left_arm_runtime_.hold_target = target;
        left_arm_runtime_.state = ArmMotionState::Holding;
        left_arm_runtime_.mode = ArmControlMode::Servo;
    }

    void RobotControlExecutor::holdRightArm()
    {
        right_arm_runtime_.state = ArmMotionState::Holding;
    }

    void RobotControlExecutor::stopLeftArm()
    {
        left_arm_.stop();
        left_arm_runtime_.state = ArmMotionState::Inactive;
        left_arm_runtime_.active_trajectory.reset();
    }

    void RobotControlExecutor::stopRightArm()
    {
        right_arm_.stop();
        right_arm_runtime_.state = ArmMotionState::Inactive;
        right_arm_runtime_.active_trajectory.reset();
    }

    void RobotControlExecutor::controlLoop()
    {
        auto next_cycle = Clock::now();

        while (!shutdown_requested_)
        {
            next_cycle += config_.control_period;
            const auto now = Clock::now();

            try
            {
                runCycle(now);
            }
            catch (const std::exception &error)
            {
                enterFault(error.what());
            }

            std::this_thread::sleep_until(next_cycle);
        }
    }

    void RobotControlExecutor::runCycle(TimePoint now)
    {
        sampleTrajectories(now);
        updateControllers();
        publishState(now);
    }

    void RobotControlExecutor::sampleTrajectories(TimePoint now)
    {
        if (left_arm_runtime_.active_trajectory)
        {
            const auto elapsed = now - left_arm_runtime_.trajectory_start_time;
            const auto point = left_arm_runtime_.active_trajectory->sample(elapsed);

            ArmTarget target;
            target.positions = point.position;
            target.velocity_limits.fill(0.2);
            left_arm_.setTarget(target.positions, target.velocity_limits);
        }

        if (right_arm_runtime_.active_trajectory)
        {
            const auto elapsed = now - right_arm_runtime_.trajectory_start_time;
            const auto point = right_arm_runtime_.active_trajectory->sample(elapsed);

            ArmTarget target;
            target.positions = point.position;
            target.velocity_limits.fill(0.2);
            right_arm_.setTarget(target.positions, target.velocity_limits);
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

    void RobotControlExecutor::publishState(TimePoint now)
    {
        RobotState state;
        state.timestamp = now;
        state.left_arm_positions = left_arm_.currentPositions();
        state.right_arm_positions = right_arm_.currentPositions();
        state.left_arm_state = left_arm_.state();
        state.right_arm_state = right_arm_.state();
        state.base_state = base_.state();

        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_state_ = state;
    }

    RobotControlExecutor::RobotState RobotControlExecutor::latestState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return robot_state_;
    }

    void RobotControlExecutor::enterFault(const std::string &reason)
    {
        state_ = State::Faulted;
    }
}
