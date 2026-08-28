#pragma once

#include "common/logger.hpp"

#include "motion/trajectory/joint_trajectory.hpp"

#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace robot::tiago
{
    class RobotControlExecutor
    {
    public:
        using Clock = std::chrono::steady_clock;
        using TimePoint = Clock::time_point;
        using Duration = Clock::duration;
        using ArmTrajectory = robot::motion::JointTrajectory<Arm::kJointCount>;
        struct Config
        {
            Duration control_period;

            // 连续命令超时。
            //
            // 当前用于：
            // - Arm Servo
            //
            // 后续用于：
            // - Base velocity
            Duration command_timeout;
        };
        enum class State
        {
            Stopped,
            Running,
            Faulted
        };

        // ========================================================
        // Arm execution state
        // ========================================================

        // Arm 当前允许使用的目标来源。
        //
        // Trajectory:
        //   Executor 每周期从 JointTrajectory 采样位置目标。
        //
        // Servo:
        //   Executor 使用外部持续提交的最新实时位置目标。
        //
        // Hold / Stop 不属于 ControlMode。
        enum class ArmControlMode
        {
            Trajectory,
            Servo
        };

        // Arm 当前运动状态。
        //
        // Inactive:
        //   当前没有 active motion source。
        //
        // Running:
        //   正在执行 Trajectory / Servo。
        //
        // Holding:
        //   显式 Hold 已生效。
        enum class ArmMotionState
        {
            Inactive,
            Running,
            Holding
        };

        // ========================================================
        // Statistics
        // ========================================================

        struct CycleStatistics
        {
            uint64_t cycle_count{0};
            Duration last_execution_time{};

            // 后续补正式统计。
            Duration max_execution_time{};
            uint64_t deadline_miss_count{0};
        };

        // ========================================================
        // Published robot state
        // ========================================================

        struct RobotState
        {
            TimePoint timestamp{};
            Arm::JointPositions left_arm_positions{};
            Arm::JointPositions right_arm_positions{};
            Gripper::FingerPositions left_gripper_positions{};
            Gripper::FingerPositions right_gripper_positions{};
            Head::JointPositions head_positions{};
            std::optional<double> torso_position{};
            ArmController::ControlState left_arm_state{ArmController::ControlState::Idle};
            ArmController::ControlState right_arm_state{ArmController::ControlState::Idle};
            BaseController::ControlState base_state{BaseController::ControlState::Idle};
        };

    public:
        RobotControlExecutor(ArmController &left_arm, ArmController &right_arm, GripperController &left_gripper, GripperController &right_gripper,
                             HeadController &head, TorsoController &torso, BaseController &base, Config config);
        ~RobotControlExecutor();
        RobotControlExecutor(const RobotControlExecutor &) = delete;
        RobotControlExecutor &operator=(const RobotControlExecutor &) = delete;

    public:
        void start();
        void shutdown();
        State state() const noexcept;

    public:
        // ========================================================
        // Arm mode
        // ========================================================

        void setLeftArmControlMode(ArmControlMode mode);
        void setRightArmControlMode(ArmControlMode mode);

    public:
        // ========================================================
        // Arm trajectory
        // ========================================================

        // velocity_limits:
        //   本次整条 trajectory 执行期间，
        //   各关节位置控制允许使用的最大速度。
        //
        // 它不是 JointTrajectoryPoint::velocity。
        //
        // 最底层 Joint 仍然会检查：
        //
        // velocity_limit <= YAML max_velocity
        void submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits);
        void submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits);

    public:
        // ========================================================
        // Arm servo
        // ========================================================

        // Servo = 实时位置目标流。
        //
        // latest target wins。
        void setLeftArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);
        void setRightArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);

    public:
        // ========================================================
        // Arm actions
        // ========================================================

        // Hold:
        //
        // 停止继续跟随当前 Trajectory / Servo，
        // 以最近一次真实反馈位置作为新的位置目标，
        // Controller 继续 Running。
        void holdLeftArm();
        void holdRightArm();

        // Stop:
        //
        // Controller::stop()，
        // Controller 回到 Idle。
        void stopLeftArm();
        void stopRightArm();

    public:
        // ========================================================
        // Gripper
        // ========================================================

        // 所有这些 public API 都只提交 mailbox。
        //
        // 真正 Controller::setTarget()
        // 在 Executor 控制线程执行。

        void setLeftGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);
        void setRightGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);

    public:
        // ========================================================
        // Head
        // ========================================================

        void setHeadTarget(const Head::JointValues &positions, const Head::JointValues &velocity_limits);

    public:
        // ========================================================
        // Torso
        // ========================================================

        void setTorsoTarget(double position, double velocity_limit);

    public:
        // ========================================================
        // Base
        // ========================================================

        // latest velocity target wins。
        //
        // command_timeout 后续会作用于这个命令流。
        void setBaseVelocity(double linear_velocity, double angular_velocity);

    public:
        RobotState latestState() const;
        CycleStatistics statistics() const;
        std::string faultMessage() const;

    private:
        // ========================================================
        // Arm internal types
        // ========================================================

        enum class ArmAction
        {
            Hold,
            Stop
        };

        // 普通 Arm 位置目标。
        //
        // Servo / Hold 使用。
        struct ArmTarget
        {
            Arm::JointValues positions{};
            Arm::JointValues velocity_limits{};
        };

        // 一条 Servo 命令必须把目标和提交时间作为一个整体传递。
        //
        // timestamp 是上层写入 Servo mailbox 时记录的时间，
        // 不是 Executor 消费 mailbox 的时间。
        struct ArmServoCommand
        {
            ArmTarget target;
            TimePoint timestamp{};
        };

        // 外部提交的一整条 trajectory 命令。
        struct ArmTrajectoryCommand
        {
            std::shared_ptr<const ArmTrajectory> trajectory;
            Arm::JointValues velocity_limits{};
        };

        // 当前真正正在执行的 trajectory。
        //
        // trajectory + velocity limits + start time
        // 属于一个不可分割的执行状态。
        struct ActiveArmTrajectory
        {
            std::shared_ptr<const ArmTrajectory> trajectory;
            Arm::JointValues velocity_limits{};
            TimePoint start_time{};
        };
        struct ArmRuntime
        {
            ArmControlMode mode{ArmControlMode::Trajectory};
            ArmMotionState state{ArmMotionState::Inactive};
            std::optional<ActiveArmTrajectory> active_trajectory;
            std::optional<ArmServoCommand> servo_target;
            std::optional<ArmTarget> hold_target;
        };

    private:
        // ========================================================
        // Other component command types
        // ========================================================

        struct GripperTarget
        {
            Gripper::FingerValues positions{};
            Gripper::FingerValues velocity_limits{};
        };
        struct HeadTarget
        {
            Head::JointValues positions{};
            Head::JointValues velocity_limits{};
        };
        struct TorsoTarget
        {
            double position{0.0};
            double velocity_limit{0.0};
        };
        struct BaseVelocityTarget
        {
            double linear_velocity{0.0};
            double angular_velocity{0.0};
        };

    private:
        // ========================================================
        // Mailbox
        // ========================================================

        struct ArmMailbox
        {
            std::optional<ArmControlMode> mode;
            std::optional<ArmAction> action;
            std::optional<ArmServoCommand> servo_target;
            std::optional<ArmTrajectoryCommand> trajectory;
        };

        // 整台机器人只有一个 CommandMailbox。
        //
        // 外部线程只在 command_mutex_ 保护下写入。
        // Executor 控制线程每周期一次性取走后独占处理。
        struct CommandMailbox
        {
            ArmMailbox left_arm;
            ArmMailbox right_arm;
            std::optional<GripperTarget> left_gripper;
            std::optional<GripperTarget> right_gripper;
            std::optional<HeadTarget> head;
            std::optional<TorsoTarget> torso;
            std::optional<BaseVelocityTarget> base_velocity;
        };

    private:
        // ========================================================
        // Robot-level executor
        // ========================================================

        void controlLoop();
        void runCycle(TimePoint now);
        void processCommands();
        void updateArmTargets(TimePoint now);
        void updateControllers();
        void publishState(TimePoint now);
        void checkDeadline(TimePoint start, TimePoint end);
        void enterFault(const std::string &reason);

    private:
        // ========================================================
        // Arm executor
        // ========================================================

        void processLeftArmCommands(ArmMailbox &commands);
        void processRightArmCommands(ArmMailbox &commands);
        void applyLeftArmHold();
        void applyRightArmHold();
        void applyLeftArmStop();
        void applyRightArmStop();

    private:
        // ========================================================
        // Other component executor commands
        // ========================================================

        void processLeftGripperCommand(const GripperTarget &command);
        void processRightGripperCommand(const GripperTarget &command);
        void processHeadCommand(const HeadTarget &command);
        void processTorsoCommand(const TorsoTarget &command);
        void processBaseVelocityCommand(const BaseVelocityTarget &command);

    private:
        // ========================================================
        // Controllers
        // ========================================================

        // Executor 不拥有 Controller。
        // 生命周期由外部负责。
        ArmController &left_arm_;
        ArmController &right_arm_;
        GripperController &left_gripper_;
        GripperController &right_gripper_;
        HeadController &head_;
        TorsoController &torso_;
        BaseController &base_;

    private:
        // ========================================================
        // Executor
        // ========================================================

        Config config_;
        std::thread control_thread_;
        std::atomic<bool> shutdown_requested_{false};
        std::atomic<State> state_{State::Stopped};

    private:
        // ========================================================
        // Command mailbox
        // ========================================================

        mutable std::mutex command_mutex_;
        CommandMailbox command_mailbox_;

    private:
        // ========================================================
        // Runtime
        // ========================================================

        // Arm 有真正的跨周期执行状态，
        // 所以需要 Runtime。
        //
        // Gripper / Head / Torso 当前没有额外 Runtime，
        // Controller 自己已经保存 latest target。
        ArmRuntime left_arm_runtime_;
        ArmRuntime right_arm_runtime_;

    private:
        // ========================================================
        // Published state
        // ========================================================

        mutable std::mutex state_mutex_;
        RobotState robot_state_{};

    private:
        // ========================================================
        // Statistics
        // ========================================================

        mutable std::mutex statistics_mutex_;
        CycleStatistics statistics_{};

    private:
        // ========================================================
        // Fault
        // ========================================================

        mutable std::mutex fault_mutex_;
        std::string fault_message_;
    };
}
