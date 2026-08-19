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
            // 后续主要用于：
            // - Arm Servo
            // - Base velocity
            Duration command_timeout;
        };
        enum class State
        {
            Stopped,
            Running,
            Faulted
        };

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
        //   正在执行 Trajectory / Servo，
        //   或已到达最终目标但 Controller 继续保持最后位置。
        //
        // Holding:
        //   显式 Hold 已生效。
        enum class ArmMotionState
        {
            Inactive,
            Running,
            Holding
        };
        struct CycleStatistics
        {
            uint64_t cycle_count{0};
            Duration last_execution_time{};

            // 后续补正式统计。
            Duration max_execution_time{};
            uint64_t deadline_miss_count{0};
        };
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
        RobotControlExecutor(ArmController &left_arm,
                             ArmController &right_arm,
                             GripperController &left_gripper,
                             GripperController &right_gripper,
                             HeadController &head,
                             TorsoController &torso,
                             BaseController &base,
                             Config config);
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

        // 显式切换 Arm 目标来源。
        //
        // 真正切换发生在 Executor 控制线程。
        //
        // Trajectory <-> Servo 切换时，
        // 旧 active source 会被清理。
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
        // 最底层 Joint 仍会再次检查：
        //   velocity_limit <= YAML max_velocity。
        void submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory,
                                     const Arm::JointValues &velocity_limits);
        void submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory,
                                      const Arm::JointValues &velocity_limits);

    public:
        // ========================================================
        // Arm servo
        // ========================================================

        // Servo 是实时位置目标流。
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
        // Other components
        // ========================================================
        //
        // 下一阶段正式接入 CommandMailbox。

        void setLeftGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);
        void setRightGripperTarget(const Gripper::FingerValues &positions,
                                   const Gripper::FingerValues &velocity_limits);
        void setHeadTarget(const Head::JointValues &positions, const Head::JointValues &velocity_limits);
        void setTorsoTarget(double position, double velocity_limit);
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

        // 外部提交的一整条 trajectory 命令。
        //
        // trajectory:
        //   轨迹本身。
        //
        // velocity_limits:
        //   这次执行允许的关节速度上限。
        struct ArmTrajectoryCommand
        {
            std::shared_ptr<const ArmTrajectory> trajectory;
            Arm::JointValues velocity_limits{};
        };

        // 当前正在执行的一整条 trajectory。
        //
        // 把 trajectory、执行速度限制和开始时间
        // 封装在一起，避免三个状态成员分散。
        struct ActiveArmTrajectory
        {
            std::shared_ptr<const ArmTrajectory> trajectory;
            Arm::JointValues velocity_limits{};
            TimePoint start_time{};
        };

        // Arm 在 Executor 中跨控制周期保存的状态。
        struct ArmRuntime
        {
            ArmControlMode mode{ArmControlMode::Trajectory};
            ArmMotionState state{ArmMotionState::Inactive};
            std::optional<ActiveArmTrajectory> active_trajectory;
            std::optional<ArmTarget> servo_target;
            std::optional<ArmTarget> hold_target;
        };

    private:
        // ========================================================
        // Mailbox
        // ========================================================

        struct ArmMailbox
        {
            std::optional<ArmControlMode> mode;
            std::optional<ArmAction> action;
            std::optional<ArmTarget> servo_target;
            std::optional<ArmTrajectoryCommand> trajectory;
        };
        struct CommandMailbox
        {
            ArmMailbox left_arm;
            ArmMailbox right_arm;
        };

    private:
        // ========================================================
        // Robot-level executor
        // ========================================================

        void controlLoop();
        void runCycle(TimePoint now);
        void processCommands();

        // 根据 ArmRuntime，
        // 更新本周期 ArmController 目标。
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
        // Controllers
        // ========================================================

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

        // 只由 Executor 控制线程访问。
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
