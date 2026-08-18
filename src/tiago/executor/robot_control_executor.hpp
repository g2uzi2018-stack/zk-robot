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
            //
            // 当前阶段暂未实现。
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
        //   Executor 每周期从 JointTrajectory 采样目标。
        //
        // Servo:
        //   Executor 使用外部持续提交的最新实时目标。
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
        //   当前没有运动控制动作。
        //
        // Running:
        //   正在执行 Trajectory / Servo。
        //
        // Holding:
        //   已执行 Hold，Controller 持续保持当前位置。
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
            // 当前字段已建立，
            // 后续再正式补统计逻辑。
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
        RobotControlExecutor(
            ArmController &left_arm,
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

        // 这里只提交模式切换请求。
        //
        // 真正修改 ArmRuntime 的操作
        // 由 Executor 控制线程完成。
        void setLeftArmControlMode(ArmControlMode mode);
        void setRightArmControlMode(ArmControlMode mode);
    public:
        // ========================================================
        // Arm trajectory
        // ========================================================

        void submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory);
        void submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory);
    public:
        // ========================================================
        // Arm servo
        // ========================================================

        // Servo 是 latest target wins。
        //
        // 外部只向 mailbox 提交最新目标，
        // 不直接修改 ArmController。
        void setLeftArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);
        void setRightArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);
    public:
        // ========================================================
        // Arm actions
        // ========================================================

        // Hold:
        //
        // 停止继续执行原来的运动来源，
        // 以最近一次真实反馈位置作为新的位置目标，
        // Controller 继续 Running 并持续保持该位置。
        //
        // public API 只负责提交请求。
        void holdLeftArm();
        void holdRightArm();
        // Stop:
        //
        // 停止 Controller，
        // Controller 回到 Idle。
        //
        // public API 同样只负责提交请求。
        void stopLeftArm();
        void stopRightArm();
    public:
        // ========================================================
        // Other components
        // ========================================================
        //
        // 这些接口已经存在，
        // 下一阶段再正式补入 CommandMailbox。

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

        // Hold / Stop 是动作，
        // 不属于 Trajectory / Servo 模式。
        enum class ArmAction
        {
            Hold,
            Stop
        };
        // 一组 Arm 位置目标。
        struct ArmTarget
        {
            Arm::JointValues positions{};
            Arm::JointValues velocity_limits{};
        };
        // Arm 在 Executor 中需要跨控制周期保存的运行状态。
        //
        // Mailbox 表示：
        //   “外部刚刚想让我做什么？”
        //
        // Runtime 表示：
        //   “Arm 现在正在做什么？”
        struct ArmRuntime
        {
            ArmControlMode mode{ArmControlMode::Trajectory};
            ArmMotionState state{ArmMotionState::Inactive};
            std::shared_ptr<const ArmTrajectory> active_trajectory;
            TimePoint trajectory_start_time{};
            std::optional<ArmTarget> servo_target;
            std::optional<ArmTarget> hold_target;
        };
    private:
        // ========================================================
        // Mailbox
        // ========================================================

        // 单条 Arm 的 mailbox。
        //
        // 这里只保存“尚未被 Executor 控制线程消费”的命令。
        struct ArmMailbox
        {
            std::optional<ArmControlMode> mode;
            std::optional<ArmAction> action;
            std::optional<ArmTarget> servo_target;
            std::shared_ptr<const ArmTrajectory> trajectory;
        };
        // 整个 RobotControlExecutor 的命令邮箱。
        //
        // 当前先放左右 Arm。
        //
        // 后续：
        // - Gripper
        // - Head
        // - Torso
        // - Base
        //
        // 继续加入这里。
        struct CommandMailbox
        {
            ArmMailbox left_arm;
            ArmMailbox right_arm;
        };
    private:
        // ========================================================
        // Robot-level executor implementation
        // ========================================================

        void controlLoop();
        void runCycle(TimePoint now);
        // 从 CommandMailbox 一次性取出本周期命令，
        // 然后交给各组件处理。
        void processCommands();
        // 根据 ArmRuntime 生成这一周期的 Arm 目标。
        //
        // Trajectory:
        //   sample trajectory。
        //
        // Servo:
        //   使用最新 servo target。
        //
        // Holding:
        //   Controller 已经保存 Hold target，
        //   这里无需重新生成目标。
        void updateArmTargets(TimePoint now);
        // 所有 Controller 按固定顺序执行一个周期。
        void updateControllers();
        void publishState(TimePoint now);
        void checkDeadline(TimePoint start, TimePoint end);
        void enterFault(const std::string &reason);
    private:
        // ========================================================
        // Arm-specific executor implementation
        // ========================================================

        void processLeftArmCommands(ArmMailbox &commands);
        void processRightArmCommands(ArmMailbox &commands);
        // 以下函数是真正执行 Hold / Stop 的地方。
        //
        // 它们只由 Executor 控制线程调用。
        void applyLeftArmHold();
        void applyRightArmHold();
        void applyLeftArmStop();
        void applyRightArmStop();
    private:
        // ========================================================
        // Controllers
        // ========================================================

        // Executor 不拥有硬件和 Controller。
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

        // 外部线程写 mailbox，
        // Executor 控制线程取 mailbox。
        mutable std::mutex command_mutex_;
        CommandMailbox command_mailbox_;
    private:
        // ========================================================
        // Runtime
        // ========================================================

        // Runtime 只由 Executor 控制线程访问。
        //
        // 因此这里本身不需要 mutex。
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
} // namespace robot::tiago
