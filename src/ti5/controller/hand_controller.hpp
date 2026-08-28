#pragma once

#include "ti5/hand/hand.hpp"

#include <cstdint>
#include <optional>

namespace robot::ti5
{

// TI5 灵巧手周期控制器。保存六通道最新目标，由 update() 统一读取状态
// 和下发 0x50。协议尚无可信的硬件停止命令，因此 pause() 只停止刷新。
class HandController final
{
public:
    enum class ControlState
    {
        Idle,
        Running,
        Failed
    };

    explicit HandController(Hand &hand);

    // 读取当前六通道位置作为初始目标。只有配置明确开放控制时才允许启动。
    void start(const Hand::SpeedValues &holding_speeds);
    void setTarget(const Hand::PositionValues &target_positions,
                   const Hand::SpeedValues &speeds);

    // 不发送未经文档确认的停止命令。
    void pause();
    void reset();
    void update();

    ControlState state() const noexcept;
    const std::optional<HandState> &currentState() const noexcept;
    const Hand::PositionValues &targetPositions() const noexcept;
    const Hand::SpeedValues &speeds() const noexcept;
    bool targetReachedRaw(std::uint16_t position_tolerance_raw) const;

private:
    Hand &hand_;
    Hand::PositionValues target_positions_{};
    Hand::SpeedValues speeds_{};
    std::optional<HandState> current_state_;
    ControlState state_{ControlState::Idle};
};

} // namespace robot::ti5
