#pragma once

#include "can/socket_can.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

namespace robot::ti5
{

// CanBus 的最小传输抽象只用于纯软件单元测试；生产构造函数仍然直接持有
// 一个 SocketCan。它不包含任何 TI5 协议语义。
class CanBusTransport
{
public:
    virtual ~CanBusTransport() = default;

    virtual void send(const robot::can::CanFrame &frame) = 0;
    virtual std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout) = 0;
};

// 本体运行时反馈的统一表示。
//
// Position Query 只填充 position_counts；CSP 反馈填充三个原始字段。时间戳
// 一律由主机在 CanBus 收到帧时使用 steady_clock 生成。
struct MotorFeedback
{
    std::uint16_t node_id{0};

    std::optional<std::int32_t> position_counts;
    std::optional<std::int16_t> speed_raw;
    std::optional<std::int16_t> current_milliamps;

    std::chrono::steady_clock::time_point timestamp{};

    enum class Source
    {
        PositionQuery,
        Csp
    };

    Source source{Source::PositionQuery};
};

using PositionFeedback = MotorFeedback;
using CspMotorFeedback = MotorFeedback;

// 一条已经完成 Discovery 的 T170C 逻辑 CAN 总线。
//
// 同一 interface 只创建一个 CanBus，多个 CanMotor 只保存对它的引用。CanBus
// 是唯一允许从 SocketCan 接收帧的层；当前阶段不创建内部线程，未来由统一
// receiver / executor 调用 collectPendingFeedback()。
class CanBus final
{
public:
    explicit CanBus(std::string interface_name);

    // 仅供纯软件单元测试的构造入口，不改变生产对象关系。
    explicit CanBus(std::unique_ptr<CanBusTransport> transport);

    ~CanBus();

    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&) = delete;
    CanBus &operator=(CanBus &&) = delete;

    void send(const robot::can::CanFrame &frame);

    // 注册由 CanMotor 使用的 node，避免把未知 8-byte 帧误当作 CSP 反馈。
    void registerNode(std::uint16_t node_id);

    // 非阻塞地收完当前 SocketCAN 接收队列；异常/未知帧不会进入状态缓存。
    void collectPendingFeedback();

    // 发送 0x08 并由本 CanBus 等待目标节点的有效响应。
    std::optional<PositionFeedback> queryPosition(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);

    // 发送 0x41 并由本 CanBus 等待目标节点的 8-byte CSP 反馈。
    std::optional<CspMotorFeedback> queryCsp(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);

    // 只读取缓存，不访问 SocketCan。
    std::optional<CspMotorFeedback> latestCspFeedback(
        std::uint16_t node_id) const;

    // 返回指定节点跨来源的最新有效反馈。
    std::optional<MotorFeedback> latestFeedback(
        std::uint16_t node_id) const;

    // 返回指定节点最新的 0x08 位置反馈。
    std::optional<PositionFeedback> latestPositionFeedback(
        std::uint16_t node_id) const;

    const std::string &interfaceName() const noexcept;

private:
    std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout);

    std::optional<MotorFeedback> decodeAndStore(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);

    void validateNodeId(std::uint16_t node_id) const;

    std::string interface_name_;
    std::unique_ptr<robot::can::SocketCan> socket_;
    std::unique_ptr<CanBusTransport> transport_;

    std::unordered_set<std::uint16_t> registered_nodes_;
    std::map<std::uint16_t, MotorFeedback> latest_feedback_;
    std::map<std::uint16_t, PositionFeedback> latest_position_feedback_;
    std::map<std::uint16_t, CspMotorFeedback> latest_csp_feedback_;
};

} // namespace robot::ti5
