#pragma once

#include "ti5/waist/waist.hpp"

#include <optional>

namespace robot::ti5
{

// Caller-driven periodic controller for five waist/fold joints.
// Call update() periodically, including while holding a target.
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

    explicit WaistController(Waist &waist);

    // Waist 必须先建立并验证当前位置控制；本函数以实测位置作为初始目标。
    void start();
    void setTarget(const Waist::JointValues &target_positions);
    void holdCurrentPosition();
    void reset();
    void update();

    ControlState state() const noexcept;
    const std::optional<WaistState> &currentState() const noexcept;
    const Waist::JointValues &targetPositions() const noexcept;
    bool targetReached(double position_tolerance_rad) const;

private:
    static Waist::JointValues requireControllablePositions(
        const WaistState &state);

    Waist &waist_;
    Waist::JointValues target_positions_{};
    std::optional<WaistState> current_state_;
    ControlState state_{ControlState::Idle};
};

} // namespace robot::ti5
