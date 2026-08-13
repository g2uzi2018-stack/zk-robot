#pragma once

#include "tiago/arm/arm.hpp"

namespace robot::tiago
{
    // 机械臂连续位置控制器。
    //
    // ArmController 负责：
    //   - 保存当前最新的关节目标
    //   - 周期读取机械臂反馈
    //   - 周期刷新当前目标到 Arm
    //   - 运行过程中允许实时更新目标
    //
    // 控制策略：
    //   - latest target wins
    //   - setTarget() 只更新目标，不直接发送 CAN 命令
    //   - update() 执行一个实际控制周期
    //
    // ArmController 不负责：
    //   - Arm 的生命周期
    //   - 控制线程
    //   - 控制周期调度
    //   - 消息队列
    //   - Motion 的 Completed / Cancelled 状态
    //   - 轨迹插值、逆运动学和路径规划
    class ArmController
    {
    public:
        enum class ControlState
        {
            Idle,
            Running,
            Failed
        };

        // 使用外部已经存在的 Arm 创建控制器。
        //
        // Arm 的生命周期由外部负责。
        explicit ArmController(Arm &arm);

        // 开始连续位置控制。
        //
        // initial_positions:
        //   初始目标关节位置。
        //
        // velocity_limits:
        //   各关节速度限制。
        //
        // Idle -> Running
        //
        // Running 或 Failed 状态下不允许再次 start。
        void start(const Arm::JointValues &initial_positions, const Arm::JointValues &velocity_limits);

        // 更新当前最新目标。
        //
        // 只允许在 Running 状态调用。
        //
        // 该函数只修改 Controller 内部目标，
        // 不直接向 Arm 发送位置命令。
        //
        // 多次调用时最后一次目标生效。
        void setTarget(const Arm::JointValues &target_positions, const Arm::JointValues &velocity_limits);

        // 停止当前连续控制。
        //
        // Running -> Idle
        //
        // 当前实现会调用 Arm::stop()。
        void stop();

        // 从 Failed 状态恢复到 Idle。
        //
        // Failed -> Idle
        void reset();

        // 执行一个控制周期。
        //
        // Running:
        //   1. 读取最新关节位置
        //   2. 将当前最新目标发送给 Arm
        //
        // Idle:
        //   只更新反馈，不发送位置命令
        //
        // Failed:
        //   不继续主动访问硬件
        //
        // 调用频率由外部负责，
        // 后续可放入 100 Hz / 300 Hz 控制循环。
        void update();

        // 获取当前 Controller 状态。
        ControlState state() const noexcept;

        // 获取最近一次 update() 保存的机械臂位置。
        const Arm::JointPositions &currentPositions() const noexcept;

        // 获取当前最新目标位置。
        const Arm::JointValues &targetPositions() const noexcept;

        // 获取当前最新速度限制。
        const Arm::JointValues &velocityLimits() const noexcept;

        // 判断当前最新目标是否已经到达。
        //
        // position_tolerance:
        //   每个关节允许的位置误差。
        //
        // 任意一个 Joint 没有反馈时返回 false。
        //
        // 该函数只查询状态，
        // 不改变 Controller 的 Running 状态。
        bool targetReached(double position_tolerance) const;

    private:
        // 被控制的机械臂。
        // 生命周期由外部负责。
        Arm &arm_;

        // 当前最新目标位置。
        Arm::JointValues target_positions_{};

        // 当前最新速度限制。
        Arm::JointValues velocity_limits_{};

        // 最近一次读取到的关节位置。
        Arm::JointPositions current_positions_{};

        // Controller 当前状态。
        ControlState state_{ControlState::Idle};
    };
}
