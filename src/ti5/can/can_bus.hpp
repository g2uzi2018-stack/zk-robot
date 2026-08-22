#pragma once

#include "can/can_frame.hpp"
#include "can/socket_can.hpp"
#include "ti5/can/can_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace robot::ti5
{

// The host-side parser context.  The wire format alone cannot distinguish
// ordinary CSP feedback from future PT feedback: both use the node ID and DLC=8.
enum class FeedbackFormat
{
    Unknown,
    Csp,
    Pt
};

template <typename T>
struct TimedValue
{
    T value{};
    std::chrono::steady_clock::time_point timestamp{};
};

// Aggregated runtime state.  Each field is updated independently by the
// feedback type that actually provides it.
struct MotorState
{
    std::uint16_t node_id{0};

    std::optional<TimedValue<std::int32_t>> position_counts;
    std::optional<TimedValue<std::int16_t>> speed_raw;
    std::optional<TimedValue<std::int16_t>> current_milliamps;

    std::uint64_t update_sequence{0};
};

struct RawFeedbackFrame
{
    robot::can::CanFrame frame{};
    std::chrono::steady_clock::time_point timestamp{};
};

// All state associated with one registered node lives together.  In
// particular, the parser format is never inferred from DLC at receive time.
struct NodeRuntimeContext
{
    FeedbackFormat feedback_format{FeedbackFormat::Unknown};
    MotorState state;

    std::optional<PositionQueryFeedback> latest_position_feedback;
    std::optional<CspFeedback> latest_csp_feedback;

    std::uint64_t unsupported_feedback_count{0};
    std::optional<RawFeedbackFrame> latest_raw_frame;
};

// The minimal transport abstraction is used by software-only tests.  The
// production constructor still owns a SocketCan.
class CanBusTransport
{
public:
    virtual ~CanBusTransport() = default;

    virtual void send(const robot::can::CanFrame &frame) = 0;
    virtual std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout) = 0;
};

// One logical T170C CAN bus.  CanBus is the only layer that receives frames;
// multiple CanMotor instances share its per-node contexts.
class CanBus final
{
public:
    explicit CanBus(std::string interface_name);
    explicit CanBus(std::unique_ptr<CanBusTransport> transport);

    ~CanBus();

    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&) = delete;
    CanBus &operator=(CanBus &&) = delete;

    void send(const robot::can::CanFrame &frame);

    // Register without changing an existing parser context.  This is useful
    // for the unambiguous 0x08 position query path.
    void registerNode(std::uint16_t node_id);

    // Explicitly select the host-side feedback parser for a node.
    void registerNode(std::uint16_t node_id, FeedbackFormat feedback_format);

    std::optional<FeedbackFormat> feedbackFormat(
        std::uint16_t node_id) const;

    // Non-blocking drain of the current receive queue.
    void collectPendingFeedback();

    // Send 0x08 and wait for that node's explicit position response.
    std::optional<PositionQueryFeedback> queryPosition(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);

    // Send 0x41 only when the node is explicitly configured for CSP.
    std::optional<CspFeedback> queryCsp(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);

    std::optional<PositionQueryFeedback> latestPositionFeedback(
        std::uint16_t node_id) const;

    std::optional<CspFeedback> latestCspFeedback(
        std::uint16_t node_id) const;

    // Canonical runtime readout: state is merged across feedback frames.
    std::optional<MotorState> latestState(std::uint16_t node_id) const;

    // Compatibility spelling for existing callers; it returns the same
    // aggregate state as latestState, never a last raw frame.
    std::optional<MotorState> latestFeedback(std::uint16_t node_id) const;

    std::optional<RawFeedbackFrame> latestRawFrame(
        std::uint16_t node_id) const;
    std::optional<RawFeedbackFrame> latestRawFeedback(
        std::uint16_t node_id) const;
    std::uint64_t unsupportedFeedbackCount(std::uint16_t node_id) const;
    std::optional<NodeRuntimeContext> nodeContext(
        std::uint16_t node_id) const;

    const std::string &interfaceName() const noexcept;

private:
    struct DecodedFeedback
    {
        enum class Kind
        {
            PositionQuery,
            Csp
        };

        Kind kind{Kind::PositionQuery};
        PositionQueryFeedback position;
        CspFeedback csp;
    };

    std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout);

    std::optional<DecodedFeedback> decodeAndStore(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);

    // Extension point only.  PT feedback semantics are intentionally not
    // implemented until the protocol documentation is complete.
    std::optional<DecodedFeedback> decodePtFeedback(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);

    NodeRuntimeContext &ensureNodeContext(std::uint16_t node_id);
    void recordUnsupportedFrame(
        NodeRuntimeContext &context,
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);
    void validateNodeId(std::uint16_t node_id) const;

    std::string interface_name_;
    std::unique_ptr<robot::can::SocketCan> socket_;
    std::unique_ptr<CanBusTransport> transport_;

    std::map<std::uint16_t, NodeRuntimeContext> node_contexts_;
};

// Compatibility names now refer to distinct protocol/runtime types rather
// than three aliases of one optional-field struct.
using PositionFeedback = PositionQueryFeedback;
using CspMotorFeedback = CspFeedback;
using MotorFeedback = MotorState;

} // namespace robot::ti5
