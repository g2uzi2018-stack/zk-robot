#include "tiago/executor/robot_control_executor.hpp"

#include <stdexcept>
#include <utility>

namespace robot::tiago
{

    RobotControlExecutor::RobotControlExecutor(
        ArmController &left_arm,
        ArmController &right_arm,
        GripperController &left_gripper,
        GripperController &right_gripper,
        HeadController &head,
        TorsoController &torso,
        BaseController &base,
        Config config)
        : left_arm_(left_arm),
          right_arm_(right_arm),
          left_gripper_(left_gripper),
          right_gripper_(right_gripper),
          head_(head),
          torso_(torso),
          base_(base),
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
        // 1.
        // 获取并处理外部线程提交的命令。
        processCommands();
        // 2.
        // 根据当前 ArmRuntime 生成本周期目标。
        updateArmTargets(now);
        // 3.
        // 所有 Controller 执行一个控制周期。
        updateControllers();
        // 4.
        // 发布本周期状态快照。
        publishState(now);
    }

    // ============================================================
    // Command mailbox
    // ============================================================

    void RobotControlExecutor::processCommands()
    {
        CommandMailbox commands;
        {
            std::lock_guard<std::mutex> lock(command_mutex_);
            // 一次性取走当前 mailbox。
            //
            // commands:
            //   本控制周期要处理的命令。
            //
            // command_mailbox_:
            //   马上恢复为空邮箱，
            //   外部线程可以继续提交下一批命令。
            commands = std::move(command_mailbox_);
            command_mailbox_ = CommandMailbox{};
        }

        // mutex 到这里已经释放。
        //
        // 后面的 Controller / Runtime 操作
        // 全部在 Executor 控制线程中完成，
        // 不占用 command_mutex_。

        processLeftArmCommands(commands.left_arm);
        processRightArmCommands(commands.right_arm);
        // 后续：
        //
        // processLeftGripperCommands(...)
        // processRightGripperCommands(...)
        // processHeadCommands(...)
        // processTorsoCommands(...)
        // processBaseCommands(...)
    }

    // ============================================================
    // Controller update
    // ============================================================

    void RobotControlExecutor::updateControllers()
    {
        // 固定执行顺序。
        //
        // 当前顺序保持第一阶段已经实测通过的设计。

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
        state.left_arm_positions = left_arm_.currentPositions();
        state.right_arm_positions = right_arm_.currentPositions();
        state.left_arm_state = left_arm_.state();
        state.right_arm_state = right_arm_.state();
        state.base_state = base_.state();
        // TODO:
        //
        // Gripper / Head / Torso 的 snapshot 字段
        // 已经存在于 RobotState，
        // 但目前尚未补完整。
        //
        // 下一阶段处理其他组件 mailbox 时一起补。

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
        // max_execution_time 当前还没有更新。
        // 后续 Statistics 阶段单独处理。

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
        // 当前 Fault 后控制循环仍然继续。
        //
        // Executor Fault 和 Controller Fault propagation
        // 后续单独设计。
    }

} // namespace robot::tiago
