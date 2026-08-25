#pragma once

#include "can/can_frame.hpp"

#include <array>
#include <chrono>
#include <cstdint>

namespace robot::can
{

// Linux SocketCAN 错误帧。error_mask 对应 linux/can/error.h 中的 CAN_ERR_* 位。
// 原始 8 字节数据同时保留，避免公共 CAN 层丢失控制器提供的诊断信息。
struct CanErrorFrame
{
    std::uint32_t error_mask{0};
    std::array<std::uint8_t, 8> data{};
};

enum class CanReceiveEventKind
{
    Data,
    Error
};

// SocketCAN 的一次接收事件。时间戳使用主机单调时钟，适合做反馈新鲜度判断。
struct CanReceiveEvent
{
    CanReceiveEventKind kind{CanReceiveEventKind::Data};
    CanFrame frame{};
    CanErrorFrame error{};
    std::chrono::steady_clock::time_point timestamp{};
};

} // namespace robot::can
