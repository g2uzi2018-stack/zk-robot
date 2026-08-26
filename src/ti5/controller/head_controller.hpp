#pragma once

#include "ti5/head/head.hpp"

#include <optional>

namespace robot::ti5
{

// TI5 头部周期位置控制器。接口与 ArmController 对齐，关节顺序保持
// neck_yaw、neck_pitch、neck_roll。
class HeadController final
{
public:
    enum class ControlState
    {
        Idle,
        Running,
        Failed
    };

    explicit HeadController(Head &head);

    // Head 必须先建立并验证当前位置控制；本函数以实测位置作为初始目标。
    void start();
    void setTarget(const Head::JointValues &target_positions);
    void holdCurrentPosition();
    void stopAndConfirm();
    void reset();
    void update();

    ControlState state() const noexcept;
    const std::optional<HeadState> &currentState() const noexcept;
    const Head::JointValues &targetPositions() const noexcept;
    bool targetReached(double position_tolerance_rad) const;

private:
    static Head::JointValues requireControllablePositions(
        const HeadState &state);

    Head &head_;
    Head::JointValues target_positions_{};
    std::optional<HeadState> current_state_;
    ControlState state_{ControlState::Idle};
};

} // namespace robot::ti5
