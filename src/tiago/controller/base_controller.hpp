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

        // 使用外部已经存在的 Base 创建控制器。
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

        // 获取当前控制器状态。
        ControlState state() const noexcept;

        // 获取当前保存的线速度目标，单位为 m/s。
        double linearVelocityTarget() const noexcept;

        // 获取当前保存的角速度目标，单位为 rad/s。
        double angularVelocityTarget() const noexcept;

    private:
        Base &base_;
        double linear_velocity_{0.0};
        double angular_velocity_{0.0};
        ControlState state_{ControlState::Idle};
    };
}
