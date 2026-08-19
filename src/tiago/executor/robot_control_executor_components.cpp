#include "tiago/executor/robot_control_executor.hpp"

namespace robot::tiago
{
    // Public setter 只写入受 mutex 保护的 mailbox；Controller 统一由 Executor
    // 控制线程修改，避免与控制周期并发访问 Controller。

    // ============================================================
    // Left Gripper mailbox
    // ============================================================

    void RobotControlExecutor::setLeftGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        // latest target wins。
        //
        // 如果 Executor 尚未消费上一条命令，
        // 新目标直接覆盖旧目标。
        command_mailbox_.left_gripper = GripperTarget{positions, velocity_limits};
    }

    // ============================================================
    // Right Gripper mailbox
    // ============================================================

    void RobotControlExecutor::setRightGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_mailbox_.right_gripper = GripperTarget{positions, velocity_limits};
    }

    // ============================================================
    // Head mailbox
    // ============================================================

    void RobotControlExecutor::setHeadTarget(const Head::JointValues &positions, const Head::JointValues &velocity_limits)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_mailbox_.head = HeadTarget{positions, velocity_limits};
    }

    // ============================================================
    // Torso mailbox
    // ============================================================

    void RobotControlExecutor::setTorsoTarget(double position, double velocity_limit)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        command_mailbox_.torso = TorsoTarget{position, velocity_limit};
    }

    // ============================================================
    // Base mailbox
    // ============================================================

    void RobotControlExecutor::setBaseVelocity(double linear_velocity, double angular_velocity)
    {
        std::lock_guard<std::mutex> lock(command_mutex_);

        // Base velocity 同样使用 latest target wins。
        //
        // command_timeout 后续会检查
        // 最后一次 Base velocity 命令的时间。
        command_mailbox_.base_velocity = BaseVelocityTarget{linear_velocity, angular_velocity};
    }

    // ============================================================
    // Left Gripper command processing
    // ============================================================

    void RobotControlExecutor::processLeftGripperCommand(const GripperTarget &command)
    {
        // 真正修改 Controller 的操作
        // 只发生在 Executor 控制线程。
        left_gripper_.setTarget(command.positions, command.velocity_limits);
    }

    // ============================================================
    // Right Gripper command processing
    // ============================================================

    void RobotControlExecutor::processRightGripperCommand(const GripperTarget &command)
    {
        right_gripper_.setTarget(command.positions, command.velocity_limits);
    }

    // ============================================================
    // Head command processing
    // ============================================================

    void RobotControlExecutor::processHeadCommand(const HeadTarget &command)
    {
        head_.setTarget(command.positions, command.velocity_limits);
    }

    // ============================================================
    // Torso command processing
    // ============================================================

    void RobotControlExecutor::processTorsoCommand(const TorsoTarget &command)
    {
        torso_.setTarget(command.position, command.velocity_limit);
    }

    // ============================================================
    // Base command processing
    // ============================================================

    void RobotControlExecutor::processBaseVelocityCommand(const BaseVelocityTarget &command)
    {
        base_.setTarget(command.linear_velocity, command.angular_velocity);
    }
}
