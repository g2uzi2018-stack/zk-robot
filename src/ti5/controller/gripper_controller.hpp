#pragma once

#include "ti5/gripper/gripper.hpp"

#include <cstdint>
#include <optional>

namespace robot::ti5
{

// TI5 灵巧手周期控制器。保存六通道最新目标，由 update() 统一读状态和
// 下发 0x50。当前没有已确认的硬件停止命令，因此只提供 pause()：它仅
// 停止后续命令刷新，不宣称灵巧手已停止、释放或断力。
class GripperController final
{
public:
    enum class ControlState
    {
        Idle,
        Running,
        Failed
    };

    explicit GripperController(Gripper &gripper);

    // 读取当前六通道位置作为初始目标。只有配置明确开放控制时才允许启动。
    void start(const Gripper::SpeedValues &holding_speeds);
    void setTarget(const Gripper::PositionValues &target_positions,
                   const Gripper::SpeedValues &speeds);

    // 仅停止 update() 继续发送 0x50，不发送未经文档确认的停止命令。
    void pause();
    void reset();
    void update();

    ControlState state() const noexcept;
    const std::optional<GripperState> &currentState() const noexcept;
    const Gripper::PositionValues &targetPositions() const noexcept;
    const Gripper::SpeedValues &speeds() const noexcept;
    bool targetReachedRaw(std::uint16_t position_tolerance_raw) const;

private:
    Gripper &gripper_;
    Gripper::PositionValues target_positions_{};
    Gripper::SpeedValues speeds_{};
    std::optional<GripperState> current_state_;
    ControlState state_{ControlState::Idle};
};

} // namespace robot::ti5
