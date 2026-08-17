#pragma once

#include "motion/trajectory/joint_trajectory.hpp"

#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
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

        // ------------------------------------------------------------
        // Executor 配置
        // ------------------------------------------------------------

        struct Config
        {
            // 整个机器人统一控制周期。
            //
            // 例如：
            //   100 ms -> 当前 Webots 测试
            //   10 ms  -> 100 Hz
            //   3.33ms -> 约 300 Hz
            Duration control_period;

            // 连续流式命令的超时时间。
            //
            // 当前主要用于 Base velocity / 后续遥操作。
            //
            // Position target 和完整 trajectory 不使用 freshness timeout，
            // 它们是保持型命令。
            Duration streaming_command_timeout;
        };

        // ------------------------------------------------------------
        // Executor 自己的生命周期
        //
        // 注意：
        // 这不是机器人运动状态。
        // Hold / Stop 不等于 Executor 停止运行。
        // ------------------------------------------------------------

        enum class State
        {
            Stopped,
            Running,
            Faulted
        };

        // ------------------------------------------------------------
        // 周期运行统计
        // ------------------------------------------------------------

        struct CycleStatistics
        {
            std::uint64_t cycle_count{0};

            Duration last_execution_time{};
            Duration max_execution_time{};

            Duration last_lateness{};
            Duration max_lateness{};

            std::uint64_t deadline_miss_count{0};
        };

        // ------------------------------------------------------------
        // 机器人状态快照
        //
        // 上层读取这里，
        // 不直接触发 CAN 查询。
        // ------------------------------------------------------------

        struct RobotState
        {
            TimePoint timestamp{};
            std::uint64_t cycle_index{0};

            Arm::JointPositions left_arm_positions{};
            Arm::JointPositions right_arm_positions{};

            Gripper::FingerPositions left_gripper_positions{};
            Gripper::FingerPositions right_gripper_positions{};

            Head::JointPositions head_positions{};

            std::optional<double> torso_position{};

            ArmController::ControlState left_arm_state{ArmController::ControlState::Idle};

            ArmController::ControlState right_arm_state{ArmController::ControlState::Idle};

            GripperController::ControlState left_gripper_state{GripperController::ControlState::Idle};

            GripperController::ControlState right_gripper_state{GripperController::ControlState::Idle};

            HeadController::ControlState head_state{HeadController::ControlState::Idle};

            TorsoController::ControlState torso_state{TorsoController::ControlState::Idle};

            BaseController::ControlState base_state{BaseController::ControlState::Idle};

            // Base 当前暂时没有里程计和真实车体位姿，
            // 因此这里只记录当前执行目标。
            double base_linear_velocity_target{0.0};
            double base_angular_velocity_target{0.0};
        };

        // ------------------------------------------------------------
        // 构造
        //
        // Executor 不拥有 Controller。
        //
        // 非常重要：
        // Executor 开始运行后，
        // 这些 Controller 只能由 Executor 线程调用。
        // ------------------------------------------------------------

        RobotControlExecutor(
            ArmController &left_arm,
            ArmController &right_arm,
            GripperController &left_gripper,
            GripperController &right_gripper,
            HeadController &head,
            TorsoController &torso,
            BaseController &base,
            Config config);

        // 释放 Executor 资源。
        ~RobotControlExecutor();

        RobotControlExecutor(const RobotControlExecutor &) = delete;
        RobotControlExecutor &operator=(const RobotControlExecutor &) = delete;

        RobotControlExecutor(RobotControlExecutor &&) = delete;
        RobotControlExecutor &operator=(RobotControlExecutor &&) = delete;

        // ------------------------------------------------------------
        // Executor 生命周期
        // ------------------------------------------------------------

        // 启动唯一机器人控制线程。
        //
        // Stopped -> Running
        void start();

        // 请求结束控制线程并等待线程退出。
        //
        // 这里使用 shutdown，而不是 stop，
        // 避免和机器人运动 stop 语义混淆。
        void shutdown();

        // 获取 Executor 当前生命周期状态。
        State state() const noexcept;

        // ------------------------------------------------------------
        // 直接位置目标
        //
        // 这些接口允许从任意上层线程调用。
        //
        // 它们只写入 Executor mailbox，
        // 不直接调用 Controller。
        //
        // 新目标统一在下一个控制周期生效。
        // latest target wins。
        // ------------------------------------------------------------

        void setLeftArmTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);

        void setRightArmTarget(const Arm::JointValues &positions, const Arm::JointValues &velocity_limits);

        void setLeftGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);

        void setRightGripperTarget(const Gripper::FingerValues &positions, const Gripper::FingerValues &velocity_limits);

        void setHeadTarget(const Head::JointValues &positions, const Head::JointValues &velocity_limits);

        void setTorsoTarget(double position, double velocity_limit);

        // ------------------------------------------------------------
        // Base 流式速度目标
        //
        // 与机械臂位置目标不同：
        // Base velocity 属于 streaming command。
        //
        // 如果上层长时间没有刷新，
        // Executor 会自动将目标置为 0。
        // ------------------------------------------------------------

        void setBaseVelocity(double linear_velocity, double angular_velocity);

        // ------------------------------------------------------------
        // 轨迹提交
        //
        // Executor 不知道轨迹是怎样生成的。
        //
        // 可以来自：
        // - 五次关节轨迹
        // - 多段轨迹
        // - 笛卡尔规划 + 逆解产生的关节轨迹
        // - 视觉抓取规划
        //
        // 对 Executor 来说全部只是 ArmTrajectory。
        // ------------------------------------------------------------

        // 提交左臂轨迹；轨迹由 Executor 在线程内按控制周期采样。
        void submitLeftArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits);

        // 提交右臂轨迹；轨迹由 Executor 在线程内按控制周期采样。
        void submitRightArmTrajectory(std::shared_ptr<const ArmTrajectory> trajectory, const Arm::JointValues &velocity_limits);

        // ------------------------------------------------------------
        // 运动控制
        // ------------------------------------------------------------

        // 暂停当前机器人运动。
        //
        // Position components:
        //   取消当前 trajectory，
        //   使用最近有效当前位置作为保持目标，
        //   Controller 保持 Running。
        //
        // Base:
        //   目标速度变为 0。
        //
        // Executor 线程继续运行。
        void hold();

        // 停止当前机器人运动。
        //
        // 取消 trajectory，
        // 并调用处于 Running 状态的 Controller::stop()。
        //
        // Executor 线程继续运行。
        void stopMotion();

        // ------------------------------------------------------------
        // 状态读取
        //
        // 只读取 Executor 已经维护好的缓存，
        // 不主动访问 CAN，不等待硬件反馈。
        // ------------------------------------------------------------

        // 获取最近一个控制周期发布的机器人状态快照。
        RobotState latestState() const;

        // 获取控制周期执行时间和 deadline miss 的统计信息。
        CycleStatistics cycleStatistics() const;

        // 获取最近一次故障的文本描述；没有故障时返回空字符串。
        std::string lastFaultMessage() const;

    private:
        // ============================================================
        // 外部命令 mailbox
        //
        // 外部线程只碰这里。
        // Controller 永远不被外部线程直接访问。
        // ============================================================

        struct ArmTarget
        {
            Arm::JointValues positions{};
            Arm::JointValues velocity_limits{};
        };

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

            TimePoint received_at{};
        };

        struct ArmTrajectoryRequest
        {
            std::shared_ptr<const ArmTrajectory> trajectory;
            Arm::JointValues velocity_limits{};
        };

        struct PendingCommands
        {
            std::optional<ArmTarget> left_arm_target;
            std::optional<ArmTarget> right_arm_target;

            std::optional<GripperTarget> left_gripper_target;
            std::optional<GripperTarget> right_gripper_target;

            std::optional<HeadTarget> head_target;
            std::optional<TorsoTarget> torso_target;

            std::optional<BaseVelocityTarget> base_velocity_target;

            std::optional<ArmTrajectoryRequest> left_arm_trajectory;
            std::optional<ArmTrajectoryRequest> right_arm_trajectory;

            bool hold_requested{false};
            bool stop_requested{false};
        };

        // ============================================================
        // 正在执行中的 Arm trajectory
        // ============================================================

        struct ActiveArmTrajectory
        {
            std::shared_ptr<const ArmTrajectory> trajectory;

            Arm::JointValues velocity_limits{};

            // 轨迹真正开始执行的 Executor cycle 时间。
            TimePoint start_time{};
        };

        // ------------------------------------------------------------
        // 控制线程
        // ------------------------------------------------------------

        void controlLoop();

        void runCycle(TimePoint cycle_start);

        // 一个周期开始时，一次性拿走外部最新命令。
        PendingCommands takePendingCommands();

        // 将本周期新命令应用到内部执行状态。
        void applyPendingCommands(PendingCommands commands, TimePoint now);

        // 当前已有 trajectory 在本周期取样。
        void sampleActiveTrajectories(TimePoint now);

        // 检查 Base 等 streaming command freshness。
        void checkCommandFreshness(TimePoint now);

        // 按固定顺序调用所有 Controller。
        void updateControllers();

        // 一个周期完成以后，
        // 发布完整 RobotState snapshot。
        void publishRobotState(TimePoint now);

        // 进入机器人级故障状态。
        void enterFault(const char *source, const std::exception &error) noexcept;

        // ------------------------------------------------------------
        // Controller helper
        //
        // 处理：
        // Idle   -> start()
        // Running -> setTarget()
        // Failed -> fault propagation
        // ------------------------------------------------------------

        void applyArmTarget(ArmController &controller, const ArmTarget &target);

        void applyGripperTarget(GripperController &controller, const GripperTarget &target);

        void applyHeadTarget(const HeadTarget &target);

        void applyTorsoTarget(const TorsoTarget &target);

        void applyBaseVelocity(double linear_velocity, double angular_velocity);

        // ------------------------------------------------------------
        // 运动语义
        // ------------------------------------------------------------

        void performHold();
        void performStopMotion();

        // ------------------------------------------------------------
        // Controllers
        //
        // 生命周期全部由外部拥有。
        // ------------------------------------------------------------

        ArmController &left_arm_;
        ArmController &right_arm_;

        GripperController &left_gripper_;
        GripperController &right_gripper_;

        HeadController &head_;
        TorsoController &torso_;

        BaseController &base_;

        // ------------------------------------------------------------
        // 配置
        // ------------------------------------------------------------

        Config config_;

        // ------------------------------------------------------------
        // Executor thread
        // ------------------------------------------------------------

        std::thread control_thread_;

        std::atomic<State> state_{State::Stopped};
        std::atomic<bool> shutdown_requested_{false};

        // ------------------------------------------------------------
        // Command mailbox
        // ------------------------------------------------------------

        mutable std::mutex command_mutex_;
        PendingCommands pending_commands_{};

        // ------------------------------------------------------------
        // Active trajectory
        //
        // 只有 Executor thread 访问，
        // 因此不需要 mutex。
        // ------------------------------------------------------------

        std::optional<ActiveArmTrajectory> left_arm_trajectory_;
        std::optional<ActiveArmTrajectory> right_arm_trajectory_;

        // ------------------------------------------------------------
        // Base command freshness
        //
        // 只有 Executor thread 修改。
        // ------------------------------------------------------------

        std::optional<TimePoint> last_base_command_time_;

        // ------------------------------------------------------------
        // RobotState snapshot
        // ------------------------------------------------------------

        mutable std::mutex state_mutex_;
        RobotState robot_state_{};

        // ------------------------------------------------------------
        // Cycle statistics
        // ------------------------------------------------------------

        mutable std::mutex statistics_mutex_;
        CycleStatistics cycle_statistics_{};

        // ------------------------------------------------------------
        // Fault information
        // ------------------------------------------------------------

        mutable std::mutex fault_mutex_;
        std::string last_fault_message_;
    };
}
