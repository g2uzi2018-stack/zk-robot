#pragma once

#include "ti5/can/can_bus.hpp"
#include "ti5/config/config.hpp"

#include <cstdint>
#include <optional>

namespace robot::ti5
{

// 一个实际 T170C 物理电机的薄运行时封装。
//
// CanMotor 不拥有 SocketCan，也不直接 receive/read；多个 Motor 通过同一个
// CanBus 共享发送和反馈缓存。Joint 方向、零偏、软限位及共享轴仲裁不属于本层。
class CanMotor final
{
public:
    CanMotor(const CanMotorConfig &config, CanBus &bus);

    std::optional<double> queryPosition();
    std::optional<double> readPosition();
    std::optional<MotorFeedback> queryCspStatus();
    std::optional<MotorFeedback> latestFeedback();

    // 只编码并发送 0x44 Position CSP；不在 main 或实机测试中调用。
    void commandPositionCsp(double position_rad);

    std::uint16_t nodeId() const noexcept;

private:
    std::uint16_t node_id_{0};
    EncoderConfig encoder_;
    JointUnit unit_{JointUnit::Radian};
    CanBus &bus_;
};

} // namespace robot::ti5
