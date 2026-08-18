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

    void RobotControlExecutor::submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory)
    {
        if (!trajectory)
        {
            throw std::invalid_argument("Left trajectory is null");
        }

        std::lock_guard<std::mutex> lock(command_mutex_);
        auto &mailbox = command_mailbox_.left_arm;
        // Hold / Stop 已经等待处理时，
        // 不允许新的运动命令覆盖动作请求。
        if (mailbox.action)
        {
            throw std::logic_error("Left arm hold/stop command is pending");
        }

        // 同一个 mailbox 周期中，
        // 不允许同时出现 Servo 和 Trajectory。
        if (mailbox.servo_target)
        {
            throw std::logic_error("Left arm has pending servo command");
        }

        mailbox.trajectory = std::move(trajectory);
    }

    void RobotControlExecutor::submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory)
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

        mailbox.trajectory = std::move(trajectory);
    }

    // ============================================================
    // Arm servo mailbox
    // ============================================================

    void RobotControlExecutor::setLeftArmServoTarget(const Arm::JointValues &positions,
                                                     const Arm::JointValues &velocity_limits)
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

        // Servo 使用 latest target wins。
        //
        // 如果 Executor 还没有消费上一次 Servo target，
        // 新目标直接覆盖旧目标。
        mailbox.servo_target = ArmTarget{positions, velocity_limits};
    }

    void RobotControlExecutor::setRightArmServoTarget(const Arm::JointValues &positions,
                                                      const Arm::JointValues &velocity_limits)
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
        // public API 只提交 Hold 请求。
        mailbox.action = ArmAction::Hold;
        // 尚未被 Executor 消费的旧运动命令失效。
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
        // 注意：
        //
        // 这里不调用 left_arm_.stop()。
        //
        // 真正的 Controller::stop()
        // 只能由 Executor 控制线程执行。
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

    void RobotControlExecutor::processLeftArmCommands(ArmMailbox &commands)
    {
        // --------------------------------------------------------
        // 1. Mode
        // --------------------------------------------------------

        if (commands.mode)
        {
            left_arm_runtime_.mode = *commands.mode;
            robot::common::logger()->info("Left arm mode changed");
            // TODO:
            //
            // 当前只是修改 mode。
            //
            // 后续需要正式实现：
            // Trajectory <-> Servo 切换时
            // 清理旧 active source。
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

            // Hold / Stop 本周期优先。
            //
            // 动作执行后不再继续处理
            // 本批次中的运动命令。
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

            left_arm_runtime_.active_trajectory = std::move(commands.trajectory);
            left_arm_runtime_.hold_target.reset();
            left_arm_runtime_.trajectory_start_time = Clock::now();
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

        if (commands.mode)
        {
            right_arm_runtime_.mode = *commands.mode;
            robot::common::logger()->info("Right arm mode changed");
            // TODO:
            //
            // 后续正式处理模式切换时
            // 清除旧 active source。
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

            right_arm_runtime_.active_trajectory = std::move(commands.trajectory);
            right_arm_runtime_.hold_target.reset();
            right_arm_runtime_.trajectory_start_time = Clock::now();
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
            right_arm_runtime_.hold_target.reset();
            right_arm_runtime_.state = ArmMotionState::Running;
        }
    }

    // ============================================================
    // Actual Hold
    // ============================================================
    //
    // Hold 语义：
    //
    // 1. 停止原来的 Trajectory / Servo 目标来源。
    // 2. 获取最近一次真实位置反馈。
    // 3. 把当前位置设为 Controller 新目标。
    // 4. Controller 继续保持 Running。
    // 5. Controller::update() 后续持续刷新该位置。
    // ============================================================

    void RobotControlExecutor::applyLeftArmHold()
    {
        if (left_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot hold left arm: controller is Failed");
        }

        // Idle 状态没有正在运行的位置 Controller，
        // 因此不能进入真正的 Holding。
        if (left_arm_.state() == ArmController::ControlState::Idle)
        {
            left_arm_runtime_.active_trajectory.reset();
            left_arm_runtime_.servo_target.reset();
            left_arm_runtime_.hold_target.reset();
            left_arm_runtime_.state = ArmMotionState::Inactive;
            robot::common::logger()->warn("Left arm hold ignored because controller is Idle");
            return;
        }

        // 使用 Controller 最近一次 update()
        // 保存的真实反馈位置。
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

        ArmTarget hold_target{hold_positions, left_arm_.velocityLimits()};
        // 真正修改 Controller。
        //
        // 此处已经处于 Executor 控制线程，
        // 因此不会由外部线程直接修改 Controller。
        left_arm_.setTarget(hold_target.positions, hold_target.velocity_limits);
        // 原来的目标来源全部失效。
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
    //
    // Stop 与 Hold 不同：
    //
    // Hold:
    //   Controller 仍然 Running。
    //   持续位置控制。
    //
    // Stop:
    //   Controller::stop()。
    //   Controller 回到 Idle。
    // ============================================================

    void RobotControlExecutor::applyLeftArmStop()
    {
        if (left_arm_.state() == ArmController::ControlState::Failed)
        {
            throw std::runtime_error("Cannot stop left arm: controller is Failed");
        }

        // ArmController::stop()
        // 只允许 Running -> Idle。
        //
        // 如果已经 Idle，
        // Executor 将重复 Stop 视为无操作。
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
    //
    // 这个函数以前叫 sampleTrajectory()。
    //
    // 但它实际上同时处理：
    // - Trajectory
    // - Servo
    //
    // 所以改名为 updateArmTargets()。
    // ============================================================

    void RobotControlExecutor::updateArmTargets(TimePoint now)
    {
        // ========================================================
        // Left trajectory
        // ========================================================

        if (left_arm_runtime_.active_trajectory)
        {
            const auto elapsed = now - left_arm_runtime_.trajectory_start_time;
            const auto point = left_arm_runtime_.active_trajectory->sample(elapsed);
            // TODO:
            //
            // 当前固定 0.2 是已知问题。
            //
            // TIAGo Arm YAML 中的 max_velocity
            // 约为 0.174532925 rad/s，
            // 因此真正测试 trajectory 前必须修正。
            //
            // 本轮只做 Executor 结构整理，
            // 暂时保持现有行为。
            Arm::JointValues velocity_limits{};
            velocity_limits.fill(0.2);
            left_arm_.setTarget(point.position, velocity_limits);
            if (point.finished)
            {
                // 不停止 Controller。
                //
                // Controller 会继续保持最后一次轨迹采样位置。
                left_arm_runtime_.active_trajectory.reset();
                robot::common::logger()->info("Left trajectory finished");
            }
        }

        // ========================================================
        // Right trajectory
        // ========================================================

        if (right_arm_runtime_.active_trajectory)
        {
            const auto elapsed = now - right_arm_runtime_.trajectory_start_time;
            const auto point = right_arm_runtime_.active_trajectory->sample(elapsed);
            // TODO:
            // 同左臂，后续处理 trajectory velocity limit。
            Arm::JointValues velocity_limits{};
            velocity_limits.fill(0.2);
            right_arm_.setTarget(point.position, velocity_limits);
            if (point.finished)
            {
                right_arm_runtime_.active_trajectory.reset();
                robot::common::logger()->info("Right trajectory finished");
            }
        }

        // ========================================================
        // Left servo
        // ========================================================

        if (left_arm_runtime_.servo_target)
        {
            left_arm_.setTarget(left_arm_runtime_.servo_target->positions,

                                left_arm_runtime_.servo_target->velocity_limits);
        }

        // ========================================================
        // Right servo
        // ========================================================

        if (right_arm_runtime_.servo_target)
        {
            right_arm_.setTarget(right_arm_runtime_.servo_target->positions,

                                 right_arm_runtime_.servo_target->velocity_limits);
        }

        // Holding 不需要在这里执行任何操作。
        //
        // applyLeftArmHold()/applyRightArmHold()
        // 已经把真实当前位置写入 Controller。
        //
        // 后面的 updateControllers()
        // 会继续刷新 Controller 保存的 Hold 目标。
    }

} // namespace robot::tiago