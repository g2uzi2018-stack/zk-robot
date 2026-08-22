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

NodeRuntimeContext &CanBus::ensureNodeContext(const std::uint16_t node_id)
{
    validateNodeId(node_id);
    auto [iterator, inserted] = node_contexts_.try_emplace(node_id);
    if (inserted)
    {
        iterator->second.state.node_id = node_id;
    }
    return iterator->second;
}

void CanBus::registerNode(const std::uint16_t node_id)
{
    static_cast<void>(ensureNodeContext(node_id));
}

void CanBus::registerNode(
    const std::uint16_t node_id,
    const FeedbackFormat feedback_format)
{
    auto &context = ensureNodeContext(node_id);
    context.feedback_format = feedback_format;
}

std::optional<FeedbackFormat> CanBus::feedbackFormat(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
    {
        return std::nullopt;
    }
    return iterator->second.feedback_format;
}

void CanBus::recordUnsupportedFrame(
    NodeRuntimeContext &context,
    const robot::can::CanFrame &frame,
    const std::chrono::steady_clock::time_point timestamp)
{
    ++context.unsupported_feedback_count;
    context.latest_raw_frame = RawFeedbackFrame{frame, timestamp};
}

std::optional<CanBus::DecodedFeedback> CanBus::decodeAndStore(
    const robot::can::CanFrame &frame,
    const std::chrono::steady_clock::time_point timestamp)
{
    // A frame from an unregistered node cannot affect runtime state.
    const auto context_iterator = node_contexts_.find(frame.id);
    if (context_iterator == node_contexts_.end())
    {
        return std::nullopt;
    }

    auto &context = context_iterator->second;

    // 0x08 has an explicit command echo and the confirmed standard response
    // is DLC=5, so it is unambiguous even when the node is not in CSP mode.
    if (isPositionQueryResponse(frame, frame.id))
    {
        try
        {
            PositionQueryFeedback feedback;
            feedback.node_id = frame.id;
            feedback.position_counts = decodePositionCounts(frame);

            context.latest_position_feedback = feedback;
            ++context.state.update_sequence;
            context.state.position_counts = TimedValue<std::int32_t>{
                feedback.position_counts,
                timestamp};

            DecodedFeedback decoded;
            decoded.kind = DecodedFeedback::Kind::PositionQuery;
            decoded.position = feedback;
            return decoded;
        }
        catch (const std::exception &)
        {
            recordUnsupportedFrame(context, frame, timestamp);
            return std::nullopt;
        }
    }

    // The parser choice is made from the registered node context.  DLC=8 is
    // deliberately not enough to select CSP because PT uses the same DLC.
    if (context.feedback_format == FeedbackFormat::Pt)
    {
        // Reserved for a future protocol-complete decodePtFeedback().
        const auto decoded = decodePtFeedback(frame, timestamp);
        if (decoded)
        {
            return decoded;
        }
        recordUnsupportedFrame(context, frame, timestamp);
        return std::nullopt;
    }

    if (context.feedback_format != FeedbackFormat::Csp ||
        !isCspFeedback(frame, frame.id))
    {
        recordUnsupportedFrame(context, frame, timestamp);
        return std::nullopt;
    }

    try
    {
        const auto feedback = decodeCspFeedback(frame);
        context.latest_csp_feedback = feedback;

        ++context.state.update_sequence;
        context.state.position_counts = TimedValue<std::int32_t>{
            feedback.position_counts,
            timestamp};
        context.state.speed_raw = TimedValue<std::int16_t>{
            feedback.speed_raw,
            timestamp};
        context.state.current_milliamps = TimedValue<std::int16_t>{
            feedback.current_milliamps,
            timestamp};

        DecodedFeedback decoded;
        decoded.kind = DecodedFeedback::Kind::Csp;
        decoded.csp = feedback;
        return decoded;
    }
    catch (const std::exception &)
    {
        recordUnsupportedFrame(context, frame, timestamp);
        return std::nullopt;
    }
}

std::optional<CanBus::DecodedFeedback> CanBus::decodePtFeedback(
    const robot::can::CanFrame &frame,
    const std::chrono::steady_clock::time_point timestamp)
{
    // PT layout, status byte semantics, and scaling are deliberately
    // unspecified in the current protocol document.  Do not guess here.
    static_cast<void>(frame);
    static_cast<void>(timestamp);
    return std::nullopt;
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

std::optional<PositionQueryFeedback> CanBus::queryPosition(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    registerNode(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    // Do not use an old queued response as the result of this query.
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
            decoded->kind == DecodedFeedback::Kind::PositionQuery &&
            decoded->position.node_id == node_id)
        {
            return decoded->position;
        }
    }
}

std::optional<CspFeedback> CanBus::queryCsp(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    // Sending 0x41 does not prove the motor's physical mode.  It only makes
    // sense when the host has explicitly selected CSP parsing for this node.
    const auto context_iterator = node_contexts_.find(node_id);
    if (context_iterator == node_contexts_.end() ||
        context_iterator->second.feedback_format != FeedbackFormat::Csp)
    {
        return std::nullopt;
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
            decoded->kind == DecodedFeedback::Kind::Csp &&
            decoded->csp.node_id == node_id)
        {
            return decoded->csp;
        }
    }
}

std::optional<PositionQueryFeedback> CanBus::latestPositionFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
    {
        return std::nullopt;
    }
    return iterator->second.latest_position_feedback;
}

std::optional<CspFeedback> CanBus::latestCspFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
    {
        return std::nullopt;
    }
    return iterator->second.latest_csp_feedback;
}

std::optional<MotorState> CanBus::latestState(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end() ||
        iterator->second.state.update_sequence == 0)
    {
        return std::nullopt;
    }
    return iterator->second.state;
}

std::optional<MotorState> CanBus::latestFeedback(
    const std::uint16_t node_id) const
{
    return latestState(node_id);
}

std::optional<RawFeedbackFrame> CanBus::latestRawFrame(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
    {
        return std::nullopt;
    }
    return iterator->second.latest_raw_frame;
}

std::optional<RawFeedbackFrame> CanBus::latestRawFeedback(
    const std::uint16_t node_id) const
{
    return latestRawFrame(node_id);
}

std::uint64_t CanBus::unsupportedFeedbackCount(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
    {
        return 0;
    }
    return iterator->second.unsupported_feedback_count;
}

std::optional<NodeRuntimeContext> CanBus::nodeContext(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    if (iterator == node_contexts_.end())
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
