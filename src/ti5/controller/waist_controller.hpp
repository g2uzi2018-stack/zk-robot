#pragma once

#include "ti5/waist/waist.hpp"

#include <optional>

namespace robot::ti5
{

// Caller-driven periodic controller for five waist/fold joints.
// Call update() periodically, including while holding a target.
// The explicit WaistPitchOnly mode keeps the other four axes at their
// entry feedback while accepting only a waist-pitch target update.
// No STOP path; failure rejects new targets without sending mode changes.
class WaistController final
{
public:
    enum class ControlState
    {
        Idle,
        Running,
        Failed
    };

    enum class ControlMode
    {
        // 腰部/折叠组的完整五轴目标。
        FullWaistFold,
        // 遥操作模式：只接受 waist_pitch 目标，其余四轴保持进入时位置。
        WaistPitchOnly
    };

    explicit WaistController(Waist &waist);

    // Waist 必须先建立并验证当前位置控制；本函数以实测位置作为初始目标。
    void start();
    // 建立腰部俯仰遥操作模式。启动时锁存五轴当前位置；后续只能通过
    // setWaistPitchTarget() 更新 node 2，其他四轴继续保持锁存目标。
    void startWaistPitchControl();
    void setTarget(const Waist::JointValues &target_positions);
    void setWaistPitchTarget(double pitch_rad);
    void holdCurrentPosition();
    void reset();
    void update();

    ControlState state() const noexcept;
    const std::optional<WaistState> &currentState() const noexcept;
    const Waist::JointValues &targetPositions() const noexcept;
    ControlMode controlMode() const noexcept;
    bool targetReached(double position_tolerance_rad) const;

private:
    void startWithMode(ControlMode mode);
    static Waist::JointValues requireControllablePositions(
        const WaistState &state);

    Waist &waist_;
    Waist::JointValues target_positions_{};
    std::optional<WaistState> current_state_;
    ControlState state_{ControlState::Idle};
    ControlMode control_mode_{ControlMode::FullWaistFold};
};

} // namespace robot::ti5
