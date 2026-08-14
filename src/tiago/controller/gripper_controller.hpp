#pragma once

#include "tiago/gripper/gripper.hpp"

namespace robot::tiago
{
    // 夹爪连续位置控制器。
    //
    // GripperController 负责：
    //   - 保存当前最新的两个 finger 目标
    //   - 周期读取夹爪反馈
    //   - 周期刷新当前目标到 Gripper
    //   - 运行过程中允许实时更新目标
    //
    // 控制策略：
    //   - latest target wins
    //   - setTarget() 只更新目标，不直接发送 CAN 命令
    //   - update() 执行一个实际控制周期
    //
    // GripperController 不负责：
    //   - Gripper 的生命周期
    //   - 控制线程和周期调度
    //   - 抓取力控制
    //   - 判断是否抓住物体
    //   - 上层抓取策略
    class GripperController
    {
    public:
        enum class ControlState
        {
            Idle,
            Running,
            Failed
        };

        // Gripper 生命周期由外部负责。
        explicit GripperController(Gripper &gripper);

        // 开始连续位置控制。
        //
        // Idle -> Running
        //
        // 不直接发送位置命令，
        // 第一次实际发送发生在 update()。
        void start(const Gripper::FingerValues &initial_positions, const Gripper::FingerValues &velocity_limits);

        // 更新当前最新目标。
        //
        // 只允许 Running 状态调用。
        // latest target wins。
        void setTarget(const Gripper::FingerValues &target_positions, const Gripper::FingerValues &velocity_limits);

        // 停止当前连续控制。
        //
        // Running -> Idle
        // 调用 Gripper::stop()。
        void stop();

        // Failed -> Idle。
        void reset();

        // 执行一个控制周期。
        //
        // Running:
        //   1. 读取两个 finger 最新位置
        //   2. 发送当前最新目标
        //
        // Idle:
        //   只更新反馈
        //
        // Failed:
        //   不继续访问硬件
        void update();

        // 获取当前控制器状态。
        ControlState state() const noexcept;

        // 最近一次 update() 保存的 finger 位置。
        const Gripper::FingerPositions &currentPositions() const noexcept;

        // 当前最新目标。
        const Gripper::FingerValues &targetPositions() const noexcept;

        // 当前速度限制。
        const Gripper::FingerValues &velocityLimits() const noexcept;

        // 判断两个 finger 是否都到达当前目标。
        //
        // position_tolerance 单位为 m。
        // 任意 finger 没有反馈时返回 false。
        bool targetReached(double position_tolerance) const;

    private:
        // 被控制的夹爪，生命周期由外部负责。
        Gripper &gripper_;

        // 当前最新的两个 finger 目标和速度限制。
        Gripper::FingerValues target_positions_{};
        Gripper::FingerValues velocity_limits_{};

        // 最近一次 update() 读取到的两个 finger 位置。
        Gripper::FingerPositions current_positions_{};

        // 控制器当前状态。
        ControlState state_{ControlState::Idle};
    };
}
