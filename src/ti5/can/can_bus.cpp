#include "ti5/can/can_bus.hpp"

#include "ti5/can/can_protocol.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace robot::ti5
{

CanBus::CanBus(std::string interface_name)
    : interface_name_(std::move(interface_name)),
      socket_(std::make_unique<robot::can::SocketCan>(interface_name_))
{
}

CanBus::CanBus(std::unique_ptr<CanBusTransport> transport)
    : transport_(std::move(transport))
{
    if (!transport_)
    {
        throw std::invalid_argument("CanBus transport must not be null");
    }
}

CanBus::~CanBus() = default;

void CanBus::send(const robot::can::CanFrame &frame)
{
    if (transport_)
    {
        transport_->send(frame);
        return;
    }
    socket_->send(frame);
}

std::optional<robot::can::CanFrame> CanBus::receive(
    const std::chrono::milliseconds timeout)
{
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus receive timeout must not be negative");
    }
    if (transport_)
    {
        return transport_->receive(timeout);
    }
    return socket_->receive(timeout);
}

void CanBus::validateNodeId(const std::uint16_t node_id) const
{
    if (node_id == 0 || node_id > 0x7FF)
    {
        throw std::invalid_argument("T170C node_id must be in range 1..2047");
    }
}

void CanBus::registerNode(const std::uint16_t node_id)
{
    validateNodeId(node_id);
    registered_nodes_.insert(node_id);
}

std::optional<MotorFeedback> CanBus::decodeAndStore(
    const robot::can::CanFrame &frame,
    const std::chrono::steady_clock::time_point timestamp)
{
    // 0x08 响应优先于 8-byte CSP 判断，避免 DLC=8 的位置响应被误判。
    if (isPositionQueryResponse(frame, frame.id))
    {
        try
        {
            MotorFeedback feedback;
            feedback.node_id = frame.id;
            feedback.position_counts = decodePositionCounts(frame);
            feedback.timestamp = timestamp;
            feedback.source = MotorFeedback::Source::PositionQuery;

            latest_feedback_[feedback.node_id] = feedback;
            latest_position_feedback_[feedback.node_id] = feedback;
            return feedback;
        }
        catch (const std::exception &)
        {
            // 协议层拒绝的帧不进入任何运行时缓存。
            return std::nullopt;
        }
    }

    // 没有注册的节点不能贡献 CSP 状态；这也过滤了其他协议的 8-byte 帧。
    if (registered_nodes_.find(frame.id) == registered_nodes_.end() ||
        !isCspFeedback(frame, frame.id))
    {
        return std::nullopt;
    }

    try
    {
        const auto decoded = decodeCspFeedback(frame);
        MotorFeedback feedback;
        feedback.node_id = decoded.node_id;
        feedback.position_counts = decoded.position_counts;
        feedback.speed_raw = decoded.speed_raw;
        feedback.current_milliamps = decoded.current_milliamps;
        feedback.timestamp = timestamp;
        feedback.source = MotorFeedback::Source::Csp;

        latest_feedback_[feedback.node_id] = feedback;
        latest_csp_feedback_[feedback.node_id] = feedback;
        return feedback;
    }
    catch (const std::exception &)
    {
        // DLC、CAN ID 或字段异常时，保留旧状态而不是写入半解析结果。
        return std::nullopt;
    }
}

void CanBus::collectPendingFeedback()
{
    while (true)
    {
        const auto frame = receive(std::chrono::milliseconds{0});
        if (!frame)
        {
            return;
        }

        static_cast<void>(decodeAndStore(
            *frame,
            std::chrono::steady_clock::now()));
    }
}

std::optional<PositionFeedback> CanBus::queryPosition(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    registerNode(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    // 旧队列中的响应不能作为本次查询结果。
    collectPendingFeedback();
    send(encodePositionQuery(node_id));

    if (timeout.count() == 0)
    {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            return std::nullopt;
        }

        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        if (remaining.count() <= 0)
        {
            remaining = std::chrono::milliseconds{1};
        }

        const auto frame = receive(remaining);
        if (!frame)
        {
            return std::nullopt;
        }

        const auto decoded = decodeAndStore(
            *frame,
            std::chrono::steady_clock::now());
        if (decoded &&
            decoded->node_id == node_id &&
            decoded->source == MotorFeedback::Source::PositionQuery)
        {
            return decoded;
        }
    }
}

std::optional<CspMotorFeedback> CanBus::queryCsp(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    registerNode(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    collectPendingFeedback();
    send(encodeCspQuery(node_id));

    if (timeout.count() == 0)
    {
        return std::nullopt;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            return std::nullopt;
        }

        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        if (remaining.count() <= 0)
        {
            remaining = std::chrono::milliseconds{1};
        }

        const auto frame = receive(remaining);
        if (!frame)
        {
            return std::nullopt;
        }

        const auto decoded = decodeAndStore(
            *frame,
            std::chrono::steady_clock::now());
        if (decoded &&
            decoded->node_id == node_id &&
            decoded->source == MotorFeedback::Source::Csp)
        {
            return decoded;
        }
    }
}

std::optional<CspMotorFeedback> CanBus::latestCspFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = latest_csp_feedback_.find(node_id);
    if (iterator == latest_csp_feedback_.end())
    {
        return std::nullopt;
    }
    return iterator->second;
}

std::optional<MotorFeedback> CanBus::latestFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = latest_feedback_.find(node_id);
    if (iterator == latest_feedback_.end())
    {
        return std::nullopt;
    }
    return iterator->second;
}

std::optional<PositionFeedback> CanBus::latestPositionFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = latest_position_feedback_.find(node_id);
    if (iterator == latest_position_feedback_.end())
    {
        return std::nullopt;
    }
    return iterator->second;
}

const std::string &CanBus::interfaceName() const noexcept
{
    return interface_name_;
}

} // namespace robot::ti5
