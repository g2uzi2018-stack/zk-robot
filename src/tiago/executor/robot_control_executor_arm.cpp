#include "tiago/executor/robot_control_executor.hpp"

#include <cstddef>
#include <stdexcept>
#include <utility>

namespace robot::tiago
{

    // ============================================================
    // Arm mode mailbox
    // ============================================================

    void RobotControlExecutor::setLeftArmControlMode(ArmControlMode mode)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_mailbox_.left_arm.mode = mode;
    }

    void RobotControlExecutor::setRightArmControlMode(ArmControlMode mode)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_mailbox_.right_arm.mode = mode;
    }

    // ============================================================
    // Arm trajectory mailbox
    // ============================================================

    void RobotControlExecutor::submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits)
    {
        if (!trajectory)
        {
            throw std::invalid_argument("Left trajectory is null");
        }
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.left_arm;

        // Hold / Stop 已经等待处理时，
        // 不允许普通运动命令覆盖安全动作。
        if (mailbox.action)
        {
            throw std::logic_error("Left arm hold/stop command is pending");
        }

        // 同一批 mailbox 中，
        // 不允许同时提交 Servo 和 Trajectory。
        if (mailbox.servo_target)
        {
            throw std::logic_error("Left arm has pending servo command");
        }

        // 新 trajectory 覆盖尚未被 Executor
        // 消费的旧 trajectory。
        mailbox.trajectory = ArmTrajectoryCommand{std::move(trajectory), velocity_limits};
    }

    void RobotControlExecutor::submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits)
    {
        if (!trajectory)
        {
            throw std::invalid_argument("Right trajectory is null");
        }
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.right_arm;
        if (mailbox.action)
        {
            throw std::logic_error("Right arm hold/stop command is pending");
        }
        if (mailbox.servo_target)
        {
            throw std::logic_error("Right arm has pending servo command");
        }
        mailbox.trajectory = ArmTrajectoryCommand{std::move(trajectory), velocity_limits};
    }

    // ============================================================
    // Arm servo mailbox
    // ============================================================

    void RobotControlExecutor::setLeftArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.left_arm;
        if (mailbox.action)
        {
            throw std::logic_error("Left arm hold/stop command is pending");
        }
        if (mailbox.trajectory)
        {
            throw std::logic_error("Left arm has pending trajectory");
        }

        // Servo latest target wins。
        mailbox.servo_target = ArmTarget{positions, velocity_limits};
    }

    void RobotControlExecutor::setRightArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.right_arm;
        if (mailbox.action)
        {
            throw std::logic_error("Right arm hold/stop command is pending");
        }
        if (mailbox.trajectory)
        {
            throw std::logic_error("Right arm has pending trajectory");
        }
        mailbox.servo_target = ArmTarget{positions, velocity_limits};
    }

    // ============================================================
    // Arm Hold / Stop mailbox
    // ============================================================

    void RobotControlExecutor::holdLeftArm()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.left_arm;
        mailbox.action = ArmAction::Hold;

        // 尚未消费的普通运动命令失效。
        mailbox.trajectory.reset();
        mailbox.servo_target.reset();
    }

    void RobotControlExecutor::holdRightArm()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.right_arm;
        mailbox.action = ArmAction::Hold;
        mailbox.trajectory.reset();
        mailbox.servo_target.reset();
    }

    void RobotControlExecutor::stopLeftArm()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.left_arm;
        mailbox.action = ArmAction::Stop;
        mailbox.trajectory.reset();
        mailbox.servo_target.reset();
    }

    void RobotControlExecutor::stopRightArm()
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.right_arm;
        mailbox.action = ArmAction::Stop;
        mailbox.trajectory.reset();
        mailbox.servo_target.reset();
    }

    // ============================================================
    // Left Arm command processing
    // ============================================================

    // 左右 Arm 都按 Mode -> Hold/Stop -> Trajectory -> Servo 的顺序消费 mailbox。
    // Hold/Stop 会提前结束本周期，避免普通运动目标覆盖安全动作。
    void RobotControlExecutor::processLeftArmCommands(ArmMailbox &commands)
    {
        // --------------------------------------------------------
        // 1. Mode
        // --------------------------------------------------------

        if (commands.mode && *commands.mode != left_arm_runtime_.mode)
        {
            left_arm_runtime_.mode = *commands.mode;

            // 模式切换必须清除旧运动 source。
            //
            // Trajectory -> Servo:
            //   清除旧 trajectory。
            //
            // Servo -> Trajectory:
            //   清除旧 servo target。
            left_arm_runtime_.active_trajectory.reset();
            left_arm_runtime_.servo_target.reset();

            // 切换 mode 本身不解除显式 Hold。
            if (left_arm_runtime_.state != ArmMotionState::Holding)
            {
                left_arm_runtime_.hold_target.reset();
                left_arm_runtime_.state = ArmMotionState::Inactive;
            }
            robot::common::logger()->info("Left arm control mode changed");
        }

        // --------------------------------------------------------
        // 2. Hold / Stop
        // --------------------------------------------------------

        if (commands.action)
        {
            if (*commands.action == ArmAction::Hold)
            {
                applyLeftArmHold();
            }
            else
            {
                applyLeftArmStop();
            }

            // 本周期 Hold / Stop 优先。
            return;
        }

        // --------------------------------------------------------
        // 3. Trajectory
        // --------------------------------------------------------

        if (commands.trajectory)
        {
            if (left_arm_runtime_.mode != ArmControlMode::Trajectory)
            {
                throw std::logic_error("Left arm is not in trajectory mode");
            }
            const auto velocity_limits = commands.trajectory->velocity_limits;
            auto trajectory = std::move(commands.trajectory->trajectory);
            left_arm_runtime_.active_trajectory = ActiveArmTrajectory{std::move(trajectory), velocity_limits, Clock::now()};

            // 新 trajectory 成为唯一运动 source。
            left_arm_runtime_.servo_target.reset();
            left_arm_runtime_.hold_target.reset();
            left_arm_runtime_.state = ArmMotionState::Running;
            robot::common::logger()->info("Left trajectory activated");
        }

        // --------------------------------------------------------
        // 4. Servo
        // --------------------------------------------------------

        if (commands.servo_target)
        {
            if (left_arm_runtime_.mode != ArmControlMode::Servo)
            {
                throw std::logic_error("Left arm is not in servo mode");
            }
            left_arm_runtime_.servo_target = std::move(commands.servo_target);

            // Servo 成为唯一运动 source。
            left_arm_runtime_.active_trajectory.reset();
            left_arm_runtime_.hold_target.reset();
            left_arm_runtime_.state = ArmMotionState::Running;
        }
    }

    // ============================================================
    // Right Arm command processing
    // ============================================================

    void RobotControlExecutor::processRightArmCommands(ArmMailbox &commands)
    {
        // --------------------------------------------------------
        // 1. Mode
        // --------------------------------------------------------

        if (commands.mode && *commands.mode != right_arm_runtime_.mode)
        {
            right_arm_runtime_.mode = *commands.mode;
            right_arm_runtime_.active_trajectory.reset();
            right_arm_runtime_.servo_target.reset();
            if (right_arm_runtime_.state != ArmMotionState::Holding)
            {
                right_arm_runtime_.hold_target.reset();
                right_arm_runtime_.state = ArmMotionState::Inactive;
            }
            robot::common::logger()->info("Right arm control mode changed");
        }

        // --------------------------------------------------------
        // 2. Hold / Stop
        // --------------------------------------------------------

        if (commands.action)
        {
            if (*commands.action == ArmAction::Hold)
            {
                applyRightArmHold();
            }
            else
            {
                applyRightArmStop();
            }

            // 本周期 Hold / Stop 优先。
            return;
        }

        // --------------------------------------------------------
        // 3. Trajectory
        // --------------------------------------------------------

        if (commands.trajectory)
        {
            if (right_arm_runtime_.mode != ArmControlMode::Trajectory)
            {
                throw std::logic_error("Right arm is not in trajectory mode");
            }
            const auto velocity_limits = commands.trajectory->velocity_limits;
            auto trajectory = std::move(commands.trajectory->trajectory);
            right_arm_runtime_.active_trajectory = ActiveArmTrajectory{std::move(trajectory), velocity_limits, Clock::now()};
            right_arm_runtime_.servo_target.reset();
            right_arm_runtime_.hold_target.reset();
            right_arm_runtime_.state = ArmMotionState::Running;
            robot::common::logger()->info("Right trajectory activated");
        }

        // --------------------------------------------------------
        // 4. Servo
        // --------------------------------------------------------

        if (commands.servo_target)
        {
            if (right_arm_runtime_.mode != ArmControlMode::Servo)
            {
                throw std::logic_error("Right arm is not in servo mode");
            }
            right_arm_runtime_.servo_target = std::move(commands.servo_target);
            right_arm_runtime_.active_trajectory.reset();
            right_arm_runtime_.hold_target.reset();
            right_arm_runtime_.state = ArmMotionState::Running;
        }
    }

    // ============================================================
    // Actual Hold
    // ============================================================

    void RobotControlExecutor::applyLeftArmHold()
    {
        if (left_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot hold left arm: controller is Failed");
        }
        if (left_arm_.state() == ArmController::ControlState::Idle)
        {
            left_arm_runtime_.active_trajectory.reset();
            left_arm_runtime_.servo_target.reset();
            left_arm_runtime_.hold_target.reset();
            left_arm_runtime_.state = ArmMotionState::Inactive;
            robot::common::logger()->warn("Left arm hold ignored because controller is Idle");
            return;
        }
        const auto &current_positions = left_arm_.currentPositions();
        Arm::JointValues hold_positions{};
        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            if (!current_positions[i])
            {
                throw std::runtime_error("Cannot hold left arm: joint feedback unavailable");
            }
            hold_positions[i] = *current_positions[i];
        }

        // Hold 继续使用 Controller 当前已有速度限制。
        //
        // Hold 的重点是把目标位置改成当前位置。
        ArmTarget hold_target{hold_positions, left_arm_.velocityLimits()};
        left_arm_.setTarget(hold_target.positions, hold_target.velocity_limits);
        left_arm_runtime_.active_trajectory.reset();
        left_arm_runtime_.servo_target.reset();
        left_arm_runtime_.hold_target = hold_target;
        left_arm_runtime_.state = ArmMotionState::Holding;
        robot::common::logger()->info("Left arm holding current position");
    }

    void RobotControlExecutor::applyRightArmHold()
    {
        if (right_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot hold right arm: controller is Failed");
        }
        if (right_arm_.state() == ArmController::ControlState::Idle)
        {
            right_arm_runtime_.active_trajectory.reset();
            right_arm_runtime_.servo_target.reset();
            right_arm_runtime_.hold_target.reset();
            right_arm_runtime_.state = ArmMotionState::Inactive;
            robot::common::logger()->warn("Right arm hold ignored because controller is Idle");
            return;
        }
        const auto &current_positions = right_arm_.currentPositions();
        Arm::JointValues hold_positions{};
        for (std::size_t i = 0; i < Arm::kJointCount; ++i)
        {
            if (!current_positions[i])
            {
                throw std::runtime_error("Cannot hold right arm: joint feedback unavailable");
            }
            hold_positions[i] = *current_positions[i];
        }
        ArmTarget hold_target{hold_positions, right_arm_.velocityLimits()};
        right_arm_.setTarget(hold_target.positions, hold_target.velocity_limits);
        right_arm_runtime_.active_trajectory.reset();
        right_arm_runtime_.servo_target.reset();
        right_arm_runtime_.hold_target = hold_target;
        right_arm_runtime_.state = ArmMotionState::Holding;
        robot::common::logger()->info("Right arm holding current position");
    }

    // ============================================================
    // Actual Stop
    // ============================================================

    void RobotControlExecutor::applyLeftArmStop()
    {
        if (left_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot stop left arm: controller is Failed");
        }
        if (left_arm_.state() == ArmController::ControlState::Running)
        {
            left_arm_.stop();
        }
        left_arm_runtime_.active_trajectory.reset();
        left_arm_runtime_.servo_target.reset();
        left_arm_runtime_.hold_target.reset();
        left_arm_runtime_.state = ArmMotionState::Inactive;
        robot::common::logger()->info("Left arm stopped");
    }

    void RobotControlExecutor::applyRightArmStop()
    {
        if (right_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot stop right arm: controller is Failed");
        }
        if (right_arm_.state() == ArmController::ControlState::Running)
        {
            right_arm_.stop();
        }
        right_arm_runtime_.active_trajectory.reset();
        right_arm_runtime_.servo_target.reset();
        right_arm_runtime_.hold_target.reset();
        right_arm_runtime_.state = ArmMotionState::Inactive;
        robot::common::logger()->info("Right arm stopped");
    }

    // ============================================================
    // Arm target update
    // ============================================================

    void RobotControlExecutor::updateArmTargets(TimePoint now)
    {
        // ========================================================
        // Left Arm
        // ========================================================

        if (left_arm_runtime_.state == ArmMotionState::Running)
        {
            if (left_arm_runtime_.mode == ArmControlMode::Trajectory)
            {
                if (left_arm_runtime_.active_trajectory)
                {
                    auto &active = *left_arm_runtime_.active_trajectory;
                    const auto elapsed = now - active.start_time;
                    const auto point = active.trajectory->sample(elapsed);

                    // point.velocity / point.acceleration
                    // 是轨迹参考状态。
                    //
                    // 当前 ArmController 是位置控制链，
                    // 所以实际下发：
                    //
                    // position       = point.position
                    // velocity_limit = 本次 trajectory 的执行限制
                    left_arm_.setTarget(point.position, active.velocity_limits);
                    if (point.finished)
                    {
                        // reached 不自动 Stop Controller。
                        //
                        // Controller 继续保持最后一个位置目标。
                        left_arm_runtime_.active_trajectory.reset();
                        robot::common::logger()->info("Left trajectory finished");
                    }
                }
            }
            else
            {
                if (left_arm_runtime_.servo_target)
                {
                    left_arm_.setTarget(left_arm_runtime_.servo_target->positions, left_arm_runtime_.servo_target->velocity_limits);
                }
            }
        }

        // ========================================================
        // Right Arm
        // ========================================================

        if (right_arm_runtime_.state == ArmMotionState::Running)
        {
            if (right_arm_runtime_.mode == ArmControlMode::Trajectory)
            {
                if (right_arm_runtime_.active_trajectory)
                {
                    auto &active = *right_arm_runtime_.active_trajectory;
                    const auto elapsed = now - active.start_time;
                    const auto point = active.trajectory->sample(elapsed);
                    right_arm_.setTarget(point.position, active.velocity_limits);
                    if (point.finished)
                    {
                        right_arm_runtime_.active_trajectory.reset();
                        robot::common::logger()->info("Right trajectory finished");
                    }
                }
            }
            else
            {
                if (right_arm_runtime_.servo_target)
                {
                    right_arm_.setTarget(right_arm_runtime_.servo_target->positions, right_arm_runtime_.servo_target->velocity_limits);
                }
            }
        }

        // Holding:
        //
        // 不需要在这里重新 setTarget。
        //
        // applyXXXHold() 已经把当前位置写入 Controller，
        // Controller::update() 会继续周期刷新。
    }
}
