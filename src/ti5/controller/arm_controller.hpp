#pragma once

#include "ti5/arm/arm.hpp"

#include <optional>

namespace robot::ti5
{

// TI5 机械臂周期位置控制器。
//
// 与 TIAGo ArmController 保持相同的分层：Controller 保存最新目标，
// setTarget() 不直接发送，update() 执行一个控制周期，轨迹规划由上层负责。
// TI5 的差异是没有已确认的速度参数接口；传入的目标必须已经是上层按控制
// 周期规划好的位置点。
class ArmController final
{
public:
    enum class ControlState
    {
        Idle,
        Running,
        Failed
    };

    explicit ArmController(Arm &arm);

    // Arm 必须已经通过 startPositionControlAtCurrentPosition() 建立并验证
    // mode=8。Controller 从最新反馈取得初始目标，避免首次 update() 跳变。
    // 本函数不发送位置命令。
    void start();

    // 只保存最新目标；非法目标不会覆盖当前目标。
    void setTarget(const Arm::JointValues &target_positions);

    // 将最新实测位置设为下一周期目标，不立即发送。
    void holdCurrentPosition();

    // 请求 Arm 的 0x02 STOP，并确认 7 轴 mode=0、fault=0。
    // STOP 不等于去使能，也不保证手臂会保持负载。
    void stopAndConfirm();

    // 只复位 Controller 的逻辑状态。调用前应先通过 Arm 完成硬件恢复。
    void reset();

    // Idle 只更新反馈；Running 更新反馈并持续刷新最新位置目标；
    // Failed 不再主动访问硬件。
    void update();

    ControlState state() const noexcept;
    const std::optional<ArmState> &currentState() const noexcept;
    const Arm::JointValues &targetPositions() const noexcept;
    bool targetReached(double position_tolerance_rad) const;

private:
    static Arm::JointValues requireControllablePositions(
        const ArmState &state);

    Arm &arm_;
    Arm::JointValues target_positions_{};
    std::optional<ArmState> current_state_;
    ControlState state_{ControlState::Idle};
};

} // namespace robot::ti5
