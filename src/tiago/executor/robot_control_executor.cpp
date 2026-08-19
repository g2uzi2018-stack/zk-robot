#include "tiago/executor/robot_control_executor.hpp"

#include <stdexcept>
#include <utility>

namespace robot::tiago
{
    RobotControlExecutor::RobotControlExecutor(ArmController &left_arm, ArmController &right_arm, GripperController &left_gripper,
                                               GripperController &right_gripper, HeadController &head, TorsoController &torso, BaseController &base,
                                               Config config)
        : left_arm_(left_arm), right_arm_(right_arm), left_gripper_(left_gripper), right_gripper_(right_gripper), head_(head), torso_(torso), base_(base),
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
            throw std::logic_error("Executor can only start from stopped state");
        }
        shutdown_requested_ = false;
        state_ = State::Running;
        robot::common::logger()->info("RobotControlExecutor started");
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
        robot::common::logger()->info("RobotControlExecutor stopped");
    }

    RobotControlExecutor::State RobotControlExecutor::state() const noexcept
    {
        return state_;
    }

    // ============================================================
    // Control loop
    // ============================================================

    void RobotControlExecutor::controlLoop()
    {
        auto next_cycle = Clock::now();
        while (!shutdown_requested_)
        {
            next_cycle += config_.control_period;
            const auto start = Clock::now();
            try
            {
                runCycle(start);
            }
            catch (const std::exception &error)
            {
                enterFault(error.what());
            }
            const auto end = Clock::now();
            checkDeadline(start, end);
            std::this_thread::sleep_until(next_cycle);
        }
    }

    void RobotControlExecutor::runCycle(TimePoint now)
    {
        // 1. 处理外部线程提交的全部命令。
        processCommands();

        // 2. Arm 有 Trajectory / Servo runtime，
        //    所以需要根据 runtime 更新本周期目标。
        updateArmTargets(now);

        // 3. 所有 Controller 执行一个控制周期。
        updateControllers();

        // 4. 发布状态快照。
        publishState(now);
    }

    // ============================================================
    // Command mailbox
    // ============================================================

    void RobotControlExecutor::processCommands()
    {
        CommandMailbox commands;
        {
            // 锁只保护 mailbox 的交换，不把 Controller 操作放在临界区内，
            // 避免外部提交线程被一个完整控制周期的执行时间阻塞。
            std::lock_guard<std::mutex> lock(command_mutex_);

            // 一次性取走整个机器人当前 mailbox。
            //
            // commands:
            //   本周期需要处理的命令。
            //
            // command_mailbox_:
            //   马上恢复为空邮箱，
            //   外部线程可以继续写下一批命令。
            commands = std::move(command_mailbox_);
            command_mailbox_ = CommandMailbox{};
        }

        // command_mutex_ 到这里已经释放。
        //
        // 从这里开始所有 Controller / Runtime
        // 操作都只发生在 Executor 控制线程。

        // 按 Arm -> Gripper -> Head -> Torso -> Base 的固定顺序消费，
        // 让同一控制周期内的命令处理顺序保持确定。

        // --------------------------------------------------------
        // Arm
        // --------------------------------------------------------

        processLeftArmCommands(commands.left_arm);
        processRightArmCommands(commands.right_arm);

        // --------------------------------------------------------
        // Gripper
        // --------------------------------------------------------

        if (commands.left_gripper)
        {
            processLeftGripperCommand(*commands.left_gripper);
        }
        if (commands.right_gripper)
        {
            processRightGripperCommand(*commands.right_gripper);
        }

        // --------------------------------------------------------
        // Head
        // --------------------------------------------------------

        if (commands.head)
        {
            processHeadCommand(*commands.head);
        }

        // --------------------------------------------------------
        // Torso
        // --------------------------------------------------------

        if (commands.torso)
        {
            processTorsoCommand(*commands.torso);
        }

        // --------------------------------------------------------
        // Base
        // --------------------------------------------------------

        if (commands.base_velocity)
        {
            processBaseVelocityCommand(*commands.base_velocity);
        }
    }

    // ============================================================
    // Controller update
    // ============================================================

    void RobotControlExecutor::updateControllers()
    {
        // 固定执行顺序。
        //
        // 与之前已经测试通过的机器人级执行顺序保持一致。

        left_arm_.update();
        right_arm_.update();
        left_gripper_.update();
        right_gripper_.update();
        head_.update();
        torso_.update();
        base_.update();
    }

    // ============================================================
    // Robot state
    // ============================================================

    void RobotControlExecutor::publishState(TimePoint now)
    {
        RobotState state;
        state.timestamp = now;

        // --------------------------------------------------------
        // Arm
        // --------------------------------------------------------

        state.left_arm_positions = left_arm_.currentPositions();
        state.right_arm_positions = right_arm_.currentPositions();
        state.left_arm_state = left_arm_.state();
        state.right_arm_state = right_arm_.state();

        // --------------------------------------------------------
        // Gripper
        // --------------------------------------------------------

        state.left_gripper_positions = left_gripper_.currentPositions();
        state.right_gripper_positions = right_gripper_.currentPositions();

        // --------------------------------------------------------
        // Head
        // --------------------------------------------------------

        state.head_positions = head_.currentPositions();

        // --------------------------------------------------------
        // Torso
        // --------------------------------------------------------

        state.torso_position = torso_.currentPosition();

        // --------------------------------------------------------
        // Base
        // --------------------------------------------------------

        state.base_state = base_.state();

        // --------------------------------------------------------
        // Publish snapshot
        // --------------------------------------------------------

        // 先组装完整快照，再一次性替换共享状态，
        // 读取线程不会观察到只更新了一部分的 RobotState。
        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_state_ = state;
    }

    RobotControlExecutor::RobotState RobotControlExecutor::latestState() const
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return robot_state_;
    }

    // ============================================================
    // Statistics
    // ============================================================

    RobotControlExecutor::CycleStatistics RobotControlExecutor::statistics() const
    {
        std::lock_guard<std::mutex> lock(statistics_mutex_);
        return statistics_;
    }

    void RobotControlExecutor::checkDeadline(TimePoint start, TimePoint end)
    {
        const auto execution = end - start;
        std::lock_guard<std::mutex> lock(statistics_mutex_);
        statistics_.cycle_count++;
        statistics_.last_execution_time = execution;

        // TODO:
        //
        // 后续补：
        // - max_execution_time
        // - lateness
        // - jitter

        if (execution > config_.control_period)
        {
            statistics_.deadline_miss_count++;
            robot::common::logger()->warn("Executor deadline miss");
        }
    }

    // ============================================================
    // Fault
    // ============================================================

    std::string RobotControlExecutor::faultMessage() const
    {
        std::lock_guard<std::mutex> lock(fault_mutex_);
        return fault_message_;
    }

    void RobotControlExecutor::enterFault(const std::string &reason)
    {
        {
            std::lock_guard<std::mutex> lock(fault_mutex_);
            fault_message_ = reason;
        }
        state_ = State::Faulted;
        robot::common::logger()->error("Executor fault: {}", reason);

        // TODO:
        //
        // Fault 后控制循环当前仍继续。
        //
        // 后续统一完成：
        // - Controller fault propagation
        // - Executor fault latching
        // - safety behavior
    }
}
