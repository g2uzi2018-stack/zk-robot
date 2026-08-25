#include "ti5/can/can_bus.hpp"

#include "ti5/can/can_protocol.hpp"

#include <array>
#include <chrono>
#include <linux/can/error.h>
#include <stdexcept>
#include <utility>

namespace robot::ti5
{

std::optional<robot::can::CanReceiveEvent> CanBusTransport::receiveEvent(
    const std::chrono::milliseconds timeout)
{
    const auto frame = receive(timeout);
    if (!frame)
    {
        return std::nullopt;
    }

    robot::can::CanReceiveEvent event;
    event.kind = robot::can::CanReceiveEventKind::Data;
    event.frame = *frame;
    event.timestamp = std::chrono::steady_clock::now();
    return event;
}

CanBus::CanBus(std::string interface_name, CanBusOptions options)
    : interface_name_(std::move(interface_name)),
      socket_(std::make_unique<robot::can::SocketCan>(
          interface_name_,
          robot::can::SocketCanOptions{
              options.use_can_filters
                  ? options.accepted_node_ids
                  : std::vector<std::uint16_t>{},
              options.receive_error_frames})),
      send_failure_threshold_(options.send_failure_threshold)
{
    if (send_failure_threshold_ == 0)
    {
        throw std::invalid_argument(
            "CanBus send_failure_threshold must be positive");
    }
    for (const auto node_id : options.accepted_node_ids)
    {
        registerNode(node_id);
    }
}

CanBus::CanBus(std::unique_ptr<CanBusTransport> transport,
               const std::size_t send_failure_threshold)
    : transport_(std::move(transport)),
      send_failure_threshold_(send_failure_threshold)
{
    if (!transport_)
    {
        throw std::invalid_argument("CanBus transport must not be null");
    }
    if (send_failure_threshold_ == 0)
    {
        throw std::invalid_argument(
            "CanBus send_failure_threshold must be positive");
    }
}

CanBus::~CanBus() = default;

void CanBus::send(const robot::can::CanFrame &frame)
{
    try
    {
        if (transport_)
        {
            transport_->send(frame);
        }
        else
        {
            socket_->send(frame);
        }
        ++health_.sent_frame_count;
        health_.consecutive_send_failures = 0;
    }
    catch (...)
    {
        ++health_.send_failure_count;
        ++health_.consecutive_send_failures;
        if (health_.consecutive_send_failures >= send_failure_threshold_)
        {
            health_.send_failure_latched = true;
        }
        throw;
    }
}

std::optional<robot::can::CanReceiveEvent> CanBus::receiveEvent(
    const std::chrono::milliseconds timeout)
{
    if (timeout.count() < 0)
    {
        throw std::invalid_argument(
            "CanBus receive timeout must not be negative");
    }
    if (transport_)
    {
        return transport_->receiveEvent(timeout);
    }
    return socket_->receiveEvent(timeout);
}

std::optional<robot::can::CanFrame> CanBus::receiveDataFrame(
    const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto remaining = timeout;

    while (true)
    {
        const auto event = receiveEvent(remaining);
        if (!event)
        {
            return std::nullopt;
        }
        if (event->kind == robot::can::CanReceiveEventKind::Error)
        {
            handleErrorFrame(event->error, event->timestamp);
        }
        else
        {
            ++health_.data_frame_count;
            health_.last_data_timestamp = event->timestamp;
            if (health_.state == CanBusState::RestartedAwaitingFeedback)
            {
                health_.state = CanBusState::Healthy;
            }
            return event->frame;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            return std::nullopt;
        }
        remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        if (remaining.count() <= 0)
        {
            remaining = std::chrono::milliseconds{1};
        }
    }
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

void CanBus::registerNode(const std::uint16_t node_id,
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
    const auto context_iterator = node_contexts_.find(frame.id);
    if (context_iterator == node_contexts_.end())
    {
        return std::nullopt;
    }

    auto &context = context_iterator->second;

    if (isPositionQueryResponse(frame, frame.id))
    {
        try
        {
            PositionQueryFeedback feedback;
            feedback.node_id = frame.id;
            feedback.position_counts = decodePositionCounts(frame);

            context.latest_position_feedback = feedback;
            ++context.state.update_sequence;
            ++context.state.position_query_sequence;
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

    constexpr std::array<DriverStatusField, 2> status_fields{
        DriverStatusField::RunMode,
        DriverStatusField::FaultBits};
    for (const auto field : status_fields)
    {
        if (!isDriverStatusQueryResponse(frame, frame.id, field))
        {
            continue;
        }

        try
        {
            const auto feedback = decodeDriverStatusFeedback(frame, field);
            ++context.state.update_sequence;
            ++context.state.status_update_sequence;

            DecodedFeedback decoded;
            decoded.status = feedback;
            if (field == DriverStatusField::RunMode)
            {
                context.latest_run_mode_feedback = feedback;
                context.state.run_mode = TimedValue<std::uint32_t>{
                    feedback.value,
                    timestamp};
                decoded.kind = DecodedFeedback::Kind::RunMode;
            }
            else
            {
                context.latest_fault_feedback = feedback;
                context.state.fault_bits = TimedValue<std::uint32_t>{
                    feedback.value,
                    timestamp};
                decoded.kind = DecodedFeedback::Kind::FaultBits;
            }
            return decoded;
        }
        catch (const std::exception &)
        {
            recordUnsupportedFrame(context, frame, timestamp);
            return std::nullopt;
        }
    }

    if (context.feedback_format == FeedbackFormat::Pt)
    {
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
        ++context.state.csp_update_sequence;
        context.state.last_csp_feedback_timestamp = timestamp;
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
    static_cast<void>(frame);
    static_cast<void>(timestamp);
    return std::nullopt;
}

void CanBus::handleErrorFrame(
    const robot::can::CanErrorFrame &error,
    const std::chrono::steady_clock::time_point timestamp)
{
    ++health_.error_frame_count;
    health_.latest_error_mask = error.error_mask;
    health_.latest_error_frame = error;
    health_.last_error_timestamp = timestamp;

    if ((error.error_mask & CAN_ERR_BUSOFF) != 0U)
    {
        health_.state = CanBusState::BusOff;
        health_.last_bus_off_timestamp = timestamp;
        return;
    }
    if ((error.error_mask & CAN_ERR_RESTARTED) != 0U)
    {
        health_.state = CanBusState::RestartedAwaitingFeedback;
        health_.last_restart_timestamp = timestamp;
        return;
    }
    if ((error.error_mask & CAN_ERR_CRTL) != 0U)
    {
        const auto flags = error.data[1];
        if ((flags & (CAN_ERR_CRTL_RX_PASSIVE |
                      CAN_ERR_CRTL_TX_PASSIVE)) != 0U)
        {
            health_.state = CanBusState::ErrorPassive;
        }
        else if ((flags & (CAN_ERR_CRTL_RX_WARNING |
                           CAN_ERR_CRTL_TX_WARNING)) != 0U)
        {
            health_.state = CanBusState::Warning;
        }
        else if ((flags & CAN_ERR_CRTL_ACTIVE) != 0U)
        {
            health_.state = CanBusState::Healthy;
        }
        return;
    }

    if (error.error_mask != 0U && health_.state == CanBusState::Healthy)
    {
        health_.state = CanBusState::Warning;
    }
}

void CanBus::handleReceiveEvent(
    const robot::can::CanReceiveEvent &event)
{
    if (event.kind == robot::can::CanReceiveEventKind::Error)
    {
        handleErrorFrame(event.error, event.timestamp);
        return;
    }

    ++health_.data_frame_count;
    health_.last_data_timestamp = event.timestamp;
    if (health_.state == CanBusState::RestartedAwaitingFeedback)
    {
        health_.state = CanBusState::Healthy;
    }
    static_cast<void>(decodeAndStore(event.frame, event.timestamp));
}

void CanBus::collectPendingFeedback()
{
    while (true)
    {
        const auto event = receiveEvent(std::chrono::milliseconds{0});
        if (!event)
        {
            return;
        }
        handleReceiveEvent(*event);
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

        const auto frame = receiveDataFrame(remaining);
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

std::optional<std::int32_t> CanBus::queryPositionLimit(
    const std::uint16_t node_id,
    const PositionLimitKind kind,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    registerNode(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    collectPendingFeedback();
    send(encodePositionLimitQuery(node_id, kind));

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

        const auto frame = receiveDataFrame(remaining);
        if (!frame)
        {
            return std::nullopt;
        }
        if (isPositionLimitQueryResponse(*frame, node_id, kind))
        {
            return decodePositionLimitCounts(*frame, kind);
        }
        static_cast<void>(decodeAndStore(
            *frame,
            std::chrono::steady_clock::now()));
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

        const auto frame = receiveDataFrame(remaining);
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

std::optional<std::uint32_t> CanBus::queryDriverStatus(
    const std::uint16_t node_id,
    const DriverStatusField field,
    const std::chrono::milliseconds timeout)
{
    validateNodeId(node_id);
    registerNode(node_id);
    if (timeout.count() < 0)
    {
        throw std::invalid_argument("CanBus query timeout must not be negative");
    }

    collectPendingFeedback();
    send(encodeDriverStatusQuery(node_id, field));

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

        const auto frame = receiveDataFrame(remaining);
        if (!frame)
        {
            return std::nullopt;
        }
        const auto decoded = decodeAndStore(
            *frame,
            std::chrono::steady_clock::now());
        const auto expected_kind =
            field == DriverStatusField::RunMode
                ? DecodedFeedback::Kind::RunMode
                : DecodedFeedback::Kind::FaultBits;
        if (decoded && decoded->kind == expected_kind &&
            decoded->status.node_id == node_id)
        {
            return decoded->status.value;
        }
    }
}

std::optional<std::uint32_t> CanBus::queryRunMode(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    return queryDriverStatus(
        node_id,
        DriverStatusField::RunMode,
        timeout);
}

std::optional<std::uint32_t> CanBus::queryFaultBits(
    const std::uint16_t node_id,
    const std::chrono::milliseconds timeout)
{
    return queryDriverStatus(
        node_id,
        DriverStatusField::FaultBits,
        timeout);
}

std::optional<PositionQueryFeedback> CanBus::latestPositionFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    return iterator == node_contexts_.end()
               ? std::nullopt
               : iterator->second.latest_position_feedback;
}

std::optional<CspFeedback> CanBus::latestCspFeedback(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    return iterator == node_contexts_.end()
               ? std::nullopt
               : iterator->second.latest_csp_feedback;
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
    return iterator == node_contexts_.end()
               ? std::nullopt
               : iterator->second.latest_raw_frame;
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
    return iterator == node_contexts_.end()
               ? 0
               : iterator->second.unsupported_feedback_count;
}

std::optional<NodeRuntimeContext> CanBus::nodeContext(
    const std::uint16_t node_id) const
{
    const auto iterator = node_contexts_.find(node_id);
    return iterator == node_contexts_.end()
               ? std::nullopt
               : std::optional<NodeRuntimeContext>{iterator->second};
}

CanBusHealth CanBus::health() const
{
    return health_;
}

const std::string &CanBus::interfaceName() const noexcept
{
    return interface_name_;
}

} // namespace robot::ti5
