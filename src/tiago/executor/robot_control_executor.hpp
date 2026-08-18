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

        using ArmTrajectory =
            robot::motion::JointTrajectory<Arm::kJointCount>;

        struct Config
        {
            Duration control_period;

            // 连续命令超时：
            // servo target
            // base velocity
            Duration command_timeout;
        };

        enum class State
        {
            Stopped,
            Running,
            Faulted
        };

        // Executor负责的目标来源模式。
        //
        // Trajectory:
        //   来自已经生成好的JointTrajectory
        //
        // Servo:
        //   来自实时关节目标输入
        enum class ArmControlMode
        {
            Trajectory,
            Servo
        };

        // 当前运动状态。
        //
        // 注意：
        // Holding不是控制模式。
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

            ArmController::ControlState left_arm_state{
                ArmController::ControlState::Idle};

            ArmController::ControlState right_arm_state{
                ArmController::ControlState::Idle};

            BaseController::ControlState base_state{
                BaseController::ControlState::Idle};
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

        RobotControlExecutor(
            const RobotControlExecutor &) = delete;

        RobotControlExecutor &operator=(
            const RobotControlExecutor &) = delete;

    public:
        void start();

        void shutdown();

        State state() const noexcept;

    public:
        // ============================
        // Arm mode
        // ============================

        // 模式切换通过Executor线程生效。
        //
        // 不直接修改运行状态。
        void setLeftArmControlMode(
            ArmControlMode mode);

        void setRightArmControlMode(
            ArmControlMode mode);

    public:
        // ============================
        // Trajectory
        // ============================

        void submitLeftArmTrajectory(
            std::shared_ptr<const ArmTrajectory> trajectory);

        void submitRightArmTrajectory(
            std::shared_ptr<const ArmTrajectory> trajectory);

    public:
        // ============================
        // Servo
        // ============================

        void setLeftArmServoTarget(
            const Arm::JointValues &positions,
            const Arm::JointValues &velocity_limits);

        void setRightArmServoTarget(
            const Arm::JointValues &positions,
            const Arm::JointValues &velocity_limits);

    public:
        void holdLeftArm();

        void holdRightArm();

        void stopLeftArm();

        void stopRightArm();

    public:
        void setLeftGripperTarget(
            const Gripper::FingerValues &positions,
            const Gripper::FingerValues &velocity_limits);

        void setRightGripperTarget(
            const Gripper::FingerValues &positions,
            const Gripper::FingerValues &velocity_limits);

        void setHeadTarget(
            const Head::JointValues &positions,
            const Head::JointValues &velocity_limits);

        void setTorsoTarget(
            double position,
            double velocity_limit);

        void setBaseVelocity(
            double linear_velocity,
            double angular_velocity);

    public:
        RobotState latestState() const;

        CycleStatistics statistics() const;

        std::string faultMessage() const;

    private:
        struct ArmTarget
        {
            Arm::JointValues positions{};

            Arm::JointValues velocity_limits{};
        };

        struct ArmRuntime
        {
            ArmControlMode mode{
                ArmControlMode::Trajectory};

            ArmMotionState state{
                ArmMotionState::Inactive};

            std::shared_ptr<const ArmTrajectory>
                active_trajectory;

            TimePoint trajectory_start_time{};

            std::optional<ArmTarget>
                servo_target;

            std::optional<ArmTarget>
                hold_target;
        };

    private:
        void controlLoop();

        void runCycle(
            TimePoint now);

        void processCommands();

        void sampleTrajectory(
            TimePoint now);

        void updateControllers();

        void publishState(
            TimePoint now);

        void checkDeadline(
            TimePoint start,
            TimePoint end);

        void enterFault(
            const std::string &reason);

    private:
        ArmController &left_arm_;

        ArmController &right_arm_;

        GripperController &left_gripper_;

        GripperController &right_gripper_;

        HeadController &head_;

        TorsoController &torso_;

        BaseController &base_;

        Config config_;

        std::thread control_thread_;

        std::atomic<bool>
            shutdown_requested_{false};

        std::atomic<State>
            state_{State::Stopped};

    private:
        // 外部线程提交命令。
        //
        // Executor线程负责消费。
        mutable std::mutex command_mutex_;

        std::optional<ArmControlMode>
            pending_left_arm_mode_;

        std::optional<ArmControlMode>
            pending_right_arm_mode_;

        std::optional<ArmTarget>
            pending_left_servo_target_;

        std::optional<ArmTarget>
            pending_right_servo_target_;

        std::shared_ptr<const ArmTrajectory>
            pending_left_trajectory_;

        std::shared_ptr<const ArmTrajectory>
            pending_right_trajectory_;

    private:
        ArmRuntime left_arm_runtime_;

        ArmRuntime right_arm_runtime_;

    private:
        mutable std::mutex state_mutex_;

        RobotState robot_state_{};

        mutable std::mutex statistics_mutex_;

        CycleStatistics statistics_{};

        mutable std::mutex fault_mutex_;

        std::string fault_message_;
    };

}