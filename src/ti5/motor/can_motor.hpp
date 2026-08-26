#pragma once

#include "ti5/can/can_bus.hpp"
#include "ti5/config/config.hpp"

#include <chrono>
#include <cstdint>
#include <optional>

namespace robot::ti5
{

struct DriverPositionLimits
{
    std::int32_t minimum_counts{0};
    std::int32_t maximum_counts{0};
    double minimum_rad{0.0};
    double maximum_rad{0.0};
};

struct DriverStatus
{
    std::uint32_t run_mode{0};
    std::uint32_t fault_bits{0};
};

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
    std::optional<CspFeedback> queryCspStatus();
    std::optional<DriverPositionLimits> queryPositionLimits();
    std::optional<DriverStatus> queryDriverStatus();
    std::optional<MotorState> latestState();
    std::optional<MotorState> latestFeedback();
    std::optional<double> readVelocity();
    std::optional<double> readCurrentAmps();

    // 只检查最近一次真正的 CSP 反馈，不会把 0x08 位置查询误当成 CSP 更新。
    bool hasFreshCspFeedback(std::chrono::milliseconds maximum_age);

    // 只编码并发送 0x44 Position CSP。调用者必须先完成独占控制、
    // 当前位置、软限位、驱动器目标范围和反馈新鲜度检查。
    void commandPositionCsp(double position_rad);

    // 发送 0x02，请求驱动器进入 STOP 运行模式。该接口不承诺去使能、
    // 释放转矩或操作抱闸，也不自动等待或验证 mode=0。
    void requestStopMode();

    std::uint16_t nodeId() const noexcept;

private:
    std::uint16_t node_id_{0};
    EncoderConfig encoder_;
    JointUnit unit_{JointUnit::Radian};
    CanBus &bus_;
};

} // namespace robot::ti5
