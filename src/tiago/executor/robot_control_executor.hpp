#pragma once

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

        using ArmTrajectory = robot::motion::JointTrajectory<Arm::kJointCount>;

        struct Config
        {
            Duration control_period;

            // 用于 Servo / Base velocity 等持续输入。
            Duration command_timeout;
        };

        enum class State
        {
            Stopped,
            Running,
            Faulted
        };

        enum class ArmControlMode
        {
            Trajectory,
            Servo
        };

        enum class ArmMotionState
        {
            Inactive,
            Running,
            Holding
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

        RobotControlExecutor(
            ArmController &left_arm,
            ArmController &right_arm,
            GripperController &left_gripper,
            GripperController &right_gripper,
            HeadController &head,
            TorsoController &torso,
            BaseController &base,
            BaseController &base,
            Config config);

        ~RobotControlExecutor();

        RobotControlExecutor(const RobotControlExecutor &) = delete;
        RobotControlExecutor &operator=(const RobotControlExecutor &) = delete;

        void start();

        void shutdown();

        State state() const noexcept;

        // =========================
        // Arm mode
        // =========================

        void setLeftArmControlMode(ArmControlMode mode);

        void setRightArmControlMode(ArmControlMode mode);

        // =========================
        // Arm trajectory mode
        //
        // 当前必须处于 Trajectory 模式
        // =========================

        void submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory);

        void submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory);

        // =========================
        // Arm servo mode
        //
        // 当前必须处于 Servo 模式
        //
        // 典型用途:
        // - 外骨骼遥操作
        // - 实时关节跟踪
        // =========================

        void setLeftArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);

        void setRightArmServoTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);

        // =========================
        // Motion control
        // =========================

        void holdLeftArm();

        void holdRightArm();

        void stopLeftArm();

        void stopRightArm();

        // =========================
        // Other components
        // =========================

        void setLeftGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);

        void setRightGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);

        void setHeadTarget(const Head::JointValues &positions, const Head::JointValues &velocity_limits);

        void setTorsoTarget(double position, double velocity_limit);

        void setBaseVelocity(double linear_velocity, double angular_velocity);

        // =========================
        // State
        // =========================

        RobotState latestState() const;

    private:
        struct ArmTarget
        {
            Arm::JointValues positions{};
            Arm::JointValues velocity_limits{};
        };

        struct ActiveTrajectory
        {
            std::shared_ptr<const ArmTrajectory> trajectory;

            TimePoint start_time{};
        };

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
        void controlLoop();

        void runCycle(TimePoint now);

        void updateControllers();

        void sampleTrajectories(TimePoint now);

        void publishState(TimePoint now);

        void checkCommandTimeout(TimePoint now);

        void applyLeftArmTarget(const ArmTarget &target);

        void applyRightArmTarget(const ArmTarget &target);

        void enterFault(const std::string &reason);

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

        std::atomic<bool> shutdown_requested_{false};

        std::atomic<State> state_{State::Stopped};

        // 外部命令缓存
        mutable std::mutex command_mutex_;

        std::optional<ArmTarget> pending_left_servo_target_;

        std::optional<ArmTarget> pending_right_servo_target_;

        std::shared_ptr<const ArmTrajectory> pending_left_trajectory_;

        std::shared_ptr<const ArmTrajectory> pending_right_trajectory_;

        ArmRuntime left_arm_runtime_;

        ArmRuntime right_arm_runtime_;

        mutable std::mutex state_mutex_;

        RobotState robot_state_{};

        std::string fault_message_;
    };

}
