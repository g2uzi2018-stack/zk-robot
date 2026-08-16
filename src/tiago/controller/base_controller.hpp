#pragma once

#include "tiago/base/base.hpp"

namespace robot::tiago
{
    // 管理底盘速度目标和控制状态，不直接处理 CAN 细节。
    class BaseController
    {
    public:
        enum class ControlState
        {
            Idle,
            Running,
            Failed
        };

        explicit BaseController(Base &base);

        // Idle -> Running，并将初始目标设为静止。
        void start();

        // 只保存最新的目标，实际发送由 update() 完成。
        void setTarget(double linear_velocity, double angular_velocity);

        // Running -> Idle，并向底盘发送停止命令。
        void stop();

        // Failed -> Idle。调用方应先确认底盘处于安全状态。
        void reset();

        // Running 状态周期发送最新目标；Idle / Failed 状态不发送命令。
        void update();

        ControlState state() const noexcept;
        double linearVelocityTarget() const noexcept;
        double angularVelocityTarget() const noexcept;

    private:
        Base &base_;
        double linear_velocity_{0.0};
        double angular_velocity_{0.0};
        ControlState state_{ControlState::Idle};
    };
}
