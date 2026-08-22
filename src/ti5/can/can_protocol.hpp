#pragma once

#include "can/can_frame.hpp"

#include <cstdint>

namespace robot::ti5
{
    // 0x08 当前位置查询响应的协议层表示。
    //
    // 这是 CAN 帧解码后的原始反馈，不包含运行时聚合状态。
    struct PositionQueryFeedback
    {
        std::uint16_t node_id{0};
        std::int32_t position_counts{0};
    };

    // 0x41 CSP 反馈帧中的已知字段。
    //
    // 0x44 的反馈在线上使用同样的布局，因此也使用这个类型。
    struct CspFeedback
    {
        // 反馈帧对应的 CAN 节点 ID。
        std::uint16_t node_id{0};

        // 当前电流，单位为 mA。
        std::int16_t current_milliamps{0};

        // 当前速度原始值，单位为 0.01 Hz（电机轴）。
        std::int16_t speed_raw{0};

        // 输出端位置计数。
        std::int32_t position_counts{0};
    };

    // 编码 0x08 读取当前位置请求帧。
    robot::can::CanFrame encodePositionQuery(std::uint16_t node_id);

    // 判断帧是否为指定节点的 0x08 当前位置响应。
    bool isPositionQueryResponse(const robot::can::CanFrame &frame,
                                 std::uint16_t expected_node_id) noexcept;

    // 解码 0x08 响应中的 little-endian int32 位置计数。
    std::int32_t decodePositionCounts(const robot::can::CanFrame &frame);

    // 编码 0x41 CSP 查询请求帧。
    robot::can::CanFrame encodeCspQuery(std::uint16_t node_id);

    // 判断帧是否为指定节点的 8 字节 CSP 反馈帧。
    // CSP 反馈不回显命令字，调用方仍需结合当前电机模式使用本判断。
    bool isCspFeedback(const robot::can::CanFrame &frame,
                      std::uint16_t expected_node_id) noexcept;

    // 解码 8 字节 CSP 反馈帧。
    CspFeedback decodeCspFeedback(const robot::can::CanFrame &frame);

    // 编码 0x44 Position CSP 目标位置帧。
    robot::can::CanFrame encodePositionCsp(std::uint16_t node_id,
                                           std::int32_t target_position_counts);
}
