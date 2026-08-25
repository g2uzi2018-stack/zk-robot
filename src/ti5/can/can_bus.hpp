#pragma once

#include "can/can_frame.hpp"
#include "can/can_receive_event.hpp"
#include "can/socket_can.hpp"
#include "ti5/can/can_protocol.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robot::ti5
{

// 普通 CSP 和以后可能加入的 PT 反馈在线上都是 DLC=8，必须由节点上下文
// 明确选择解析方式，不能只根据帧长度判断。
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

// 一个电机节点的聚合状态。不同字段可以来自不同种类的反馈，所以每个字段
// 都保存自己的时间戳。csp_update_sequence 只由真正的 8 字节 CSP 反馈更新。
struct MotorState
{
    std::uint16_t node_id{0};

    std::optional<TimedValue<std::int32_t>> position_counts;
    std::optional<TimedValue<std::int16_t>> speed_raw;
    std::optional<TimedValue<std::int16_t>> current_milliamps;
    std::optional<TimedValue<std::uint32_t>> run_mode;
    std::optional<TimedValue<std::uint32_t>> fault_bits;

    std::uint64_t update_sequence{0};
    std::uint64_t position_query_sequence{0};
    std::uint64_t csp_update_sequence{0};
    std::uint64_t status_update_sequence{0};
    std::optional<std::chrono::steady_clock::time_point>
        last_csp_feedback_timestamp;
};

struct RawFeedbackFrame
{
    robot::can::CanFrame frame{};
    std::chrono::steady_clock::time_point timestamp{};
};

struct NodeRuntimeContext
{
    FeedbackFormat feedback_format{FeedbackFormat::Unknown};
    MotorState state;

    std::optional<PositionQueryFeedback> latest_position_feedback;
    std::optional<CspFeedback> latest_csp_feedback;
    std::optional<DriverStatusFeedback> latest_run_mode_feedback;
    std::optional<DriverStatusFeedback> latest_fault_feedback;

    std::uint64_t unsupported_feedback_count{0};
    std::optional<RawFeedbackFrame> latest_raw_frame;
};

enum class CanBusState
{
    Healthy,
    Warning,
    ErrorPassive,
    BusOff,
    RestartedAwaitingFeedback
};

// 一条 Linux CAN 接口的健康快照。这里只报告事实，不自动恢复运动。
struct CanBusHealth
{
    CanBusState state{CanBusState::Healthy};
    std::uint64_t data_frame_count{0};
    std::uint64_t error_frame_count{0};
    std::uint64_t sent_frame_count{0};
    std::uint64_t send_failure_count{0};
    std::size_t consecutive_send_failures{0};
    bool send_failure_latched{false};
    std::uint32_t latest_error_mask{0};
    std::optional<std::chrono::steady_clock::time_point> last_data_timestamp;
    std::optional<std::chrono::steady_clock::time_point> last_error_timestamp;
    std::optional<std::chrono::steady_clock::time_point> last_bus_off_timestamp;
    std::optional<std::chrono::steady_clock::time_point> last_restart_timestamp;
    std::optional<robot::can::CanErrorFrame> latest_error_frame;
};

struct CanBusOptions
{
    std::vector<std::uint16_t> accepted_node_ids;
    bool use_can_filters{false};
    bool receive_error_frames{true};
    std::size_t send_failure_threshold{30};
};

// 软件自动测试使用的最小传输接口。现有只提供普通帧的测试替身仍可实现
// receive()；需要验证错误状态时可以覆盖 receiveEvent()。
class CanBusTransport
{
public:
    virtual ~CanBusTransport() = default;

    virtual void send(const robot::can::CanFrame &frame) = 0;
    virtual std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout) = 0;

    virtual std::optional<robot::can::CanReceiveEvent> receiveEvent(
        std::chrono::milliseconds timeout);
};

// 一条 TI5 逻辑 CAN 总线。CanBus 是该接口唯一的接收者，同一总线上的
// 多个 CanMotor 共享它保存的节点状态和总线健康状态。
class CanBus final
{
public:
    explicit CanBus(std::string interface_name,
                    CanBusOptions options = {});
    explicit CanBus(std::unique_ptr<CanBusTransport> transport,
                    std::size_t send_failure_threshold = 30);

    ~CanBus();

    CanBus(const CanBus &) = delete;
    CanBus &operator=(const CanBus &) = delete;
    CanBus(CanBus &&) = delete;
    CanBus &operator=(CanBus &&) = delete;

    void send(const robot::can::CanFrame &frame);

    void registerNode(std::uint16_t node_id);
    void registerNode(std::uint16_t node_id,
                      FeedbackFormat feedback_format);
    std::optional<FeedbackFormat> feedbackFormat(
        std::uint16_t node_id) const;

    // 不等待新数据，读取当前队列中的普通反馈和 CAN 错误事件。
    void collectPendingFeedback();

    std::optional<PositionQueryFeedback> queryPosition(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);
    std::optional<std::int32_t> queryPositionLimit(
        std::uint16_t node_id,
        PositionLimitKind kind,
        std::chrono::milliseconds timeout);
    std::optional<CspFeedback> queryCsp(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);
    std::optional<std::uint32_t> queryRunMode(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);
    std::optional<std::uint32_t> queryFaultBits(
        std::uint16_t node_id,
        std::chrono::milliseconds timeout);

    std::optional<PositionQueryFeedback> latestPositionFeedback(
        std::uint16_t node_id) const;
    std::optional<CspFeedback> latestCspFeedback(
        std::uint16_t node_id) const;
    std::optional<MotorState> latestState(std::uint16_t node_id) const;
    std::optional<MotorState> latestFeedback(std::uint16_t node_id) const;
    std::optional<RawFeedbackFrame> latestRawFrame(
        std::uint16_t node_id) const;
    std::optional<RawFeedbackFrame> latestRawFeedback(
        std::uint16_t node_id) const;
    std::uint64_t unsupportedFeedbackCount(std::uint16_t node_id) const;
    std::optional<NodeRuntimeContext> nodeContext(
        std::uint16_t node_id) const;

    CanBusHealth health() const;
    const std::string &interfaceName() const noexcept;

private:
    struct DecodedFeedback
    {
        enum class Kind
        {
            PositionQuery,
            Csp,
            RunMode,
            FaultBits
        };

        Kind kind{Kind::PositionQuery};
        PositionQueryFeedback position;
        CspFeedback csp;
        DriverStatusFeedback status;
    };

    std::optional<robot::can::CanReceiveEvent> receiveEvent(
        std::chrono::milliseconds timeout);
    std::optional<robot::can::CanFrame> receiveDataFrame(
        std::chrono::milliseconds timeout);

    std::optional<DecodedFeedback> decodeAndStore(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);
    std::optional<DecodedFeedback> decodePtFeedback(
        const robot::can::CanFrame &frame,
        std::chrono::steady_clock::time_point timestamp);
    std::optional<std::uint32_t> queryDriverStatus(
        std::uint16_t node_id,
        DriverStatusField field,
        std::chrono::milliseconds timeout);

    void handleReceiveEvent(const robot::can::CanReceiveEvent &event);
    void handleErrorFrame(const robot::can::CanErrorFrame &error,
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
    std::size_t send_failure_threshold_{30};
    CanBusHealth health_;
    std::map<std::uint16_t, NodeRuntimeContext> node_contexts_;
};

using PositionFeedback = PositionQueryFeedback;
using CspMotorFeedback = CspFeedback;
using MotorFeedback = MotorState;

} // namespace robot::ti5
