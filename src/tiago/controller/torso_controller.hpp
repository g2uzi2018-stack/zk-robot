#pragma once

#include "tiago/torso/torso.hpp"

#include <optional>

namespace robot::tiago
{
    // 躯干升降连续位置控制器。
    //
    // TorsoController 负责：
    //   - 保存当前最新升降目标
    //   - 周期读取 Torso 反馈
    //   - 周期刷新当前目标
    //   - Running 状态下允许实时更新目标
    //
    // TorsoController 不负责：
    //   - Torso 生命周期
    //   - enable / disable
    //   - 控制线程
    //   - 控制周期调度
    //   - 轨迹规划
    class TorsoController
    {
    public:
        enum class ControlState
        {
            Idle,
            Running,
            Failed
        };

        // Torso 生命周期由外部负责。
        explicit TorsoController(Torso &torso);

        // Idle -> Running。
        // 不立即发送位置命令。
        void start(double initial_position, double velocity_limit);

        // Running 状态下更新最新目标。
        // 不立即发送位置命令。
        void setTarget(double target_position, double velocity_limit);

        // Running -> Idle。
        // 调用 Torso::stop()。
        void stop();

        // Failed -> Idle。
        void reset();

        // 执行一个控制周期。
        //
        // Running:
        //   读取反馈 + 刷新最新目标
        //
        // Idle:
        //   只读取反馈
        //
        // Failed:
        //   不继续访问硬件
        void update();

        // 获取当前控制器状态。
        ControlState state() const noexcept;

        // 获取最近一次 update() 保存的位置。
        const std::optional<double> &currentPosition() const noexcept;

        // 获取当前目标位置。
        double targetPosition() const noexcept;

        // 获取当前速度限制。
        double velocityLimit() const noexcept;

        // 判断当前目标是否已经到达。
        bool targetReached(double position_tolerance) const;

    private:
        // 被控制的 Torso，生命周期由外部负责。
        Torso &torso_;

        // 当前目标、速度限制和反馈位置。
        double target_position_{0.0};
        double velocity_limit_{0.0};
        std::optional<double> current_position_{};

        // 控制器当前状态。
        ControlState state_{ControlState::Idle};
    };
}
