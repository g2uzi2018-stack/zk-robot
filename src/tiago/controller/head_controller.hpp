#pragma once

#include "tiago/head/head.hpp"

namespace robot::tiago
{
    // 头部连续位置控制器。
    //
    // HeadController 负责：
    //   - 保存当前最新的两个头部关节目标
    //   - 周期读取 Head 反馈
    //   - 周期刷新当前目标到 Head
    //   - 运行过程中允许实时更新目标
    //
    // 控制策略：
    //   - latest target wins
    //   - setTarget() 只更新目标，不直接发送 CAN 命令
    //   - update() 执行一个实际控制周期
    //
    // HeadController 不负责：
    //   - Head 的生命周期
    //   - 控制线程和周期调度
    //   - 轨迹规划
    //   - 视觉跟踪
    //   - 上层头部动作策略
    class HeadController
    {
    public:
        enum class ControlState
        {
            Idle,
            Running,
            Failed
        };

        // Head 生命周期由外部负责。
        explicit HeadController(Head &head);

        // 开始连续位置控制。
        //
        // Idle -> Running
        //
        // 不直接发送位置命令，
        // 第一次实际发送发生在 update()。
        void start(const Head::JointValues &initial_positions, const Head::JointValues &velocity_limits);

        // 更新当前最新目标。
        //
        // 只允许 Running 状态调用。
        // latest target wins。
        void setTarget(const Head::JointValues &target_positions, const Head::JointValues &velocity_limits);

        // 停止当前连续控制。
        //
        // Running -> Idle
        // 调用 Head::stop()。
        void stop();

        // Failed -> Idle。
        void reset();

        // 执行一个控制周期。
        //
        // Running:
        //   1. 读取两个 Joint 最新位置
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

        // 获取最近一次 update() 保存的位置。
        const Head::JointPositions &currentPositions() const noexcept;

        // 获取当前目标位置。
        const Head::JointValues &targetPositions() const noexcept;

        // 获取当前速度限制。
        const Head::JointValues &velocityLimits() const noexcept;

        // 判断两个 Joint 是否都到达当前目标。
        //
        // position_tolerance 单位 rad。
        // 任意 Joint 没有反馈时返回 false。
        bool targetReached(double position_tolerance) const;

    private:
        // 被控制的 Head，生命周期由外部负责。
        Head &head_;

        // 当前最新目标和速度限制。
        Head::JointValues target_positions_{};
        Head::JointValues velocity_limits_{};

        // 最近一次 update() 读取的位置。
        Head::JointPositions current_positions_{};

        // 控制器当前状态。
        ControlState state_{ControlState::Idle};
    };
}
