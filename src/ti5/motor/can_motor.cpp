#include "ti5/motor/can_motor.hpp"

#include "ti5/can/can_protocol.hpp"
#include "ti5/can/encoder_conversion.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::ti5
{

CanMotor::CanMotor(const CanMotorConfig &config, CanBus &bus)
    : node_id_(config.node_id),
      encoder_(config.encoder),
      unit_(config.unit),
      bus_(bus)
{
    if (unit_ != JointUnit::Radian)
    {
        throw std::invalid_argument("T170C CanMotor currently requires radian unit");
    }
    if (encoder_.type != "dual" || encoder_.position_reference != "output")
    {
        throw std::invalid_argument("T170C CanMotor requires dual output encoder");
    }
    if (encoder_.counts_per_output_revolution == 0 ||
        !(encoder_.gear_ratio > 0.0) ||
        !std::isfinite(encoder_.gear_ratio))
    {
        throw std::invalid_argument("T170C encoder configuration is invalid");
    }

    // The current CanMotor implementation is CSP-only.  Registering the
    // parser context explicitly prevents a future PT DLC=8 frame from being
    // decoded as CSP by accident.
    bus_.registerNode(node_id_, FeedbackFormat::Csp);
}

std::optional<double> CanMotor::queryPosition()
{
    const auto feedback = bus_.queryPosition(
        node_id_,
        std::chrono::milliseconds{50});
    // position_counts=0 是合法的零位反馈，不能把数值 0 当成“无反馈”。
    if (!feedback)
    {
        return std::nullopt;
    }
    return positionCountsToRadians(
        feedback->position_counts,
        encoder_.counts_per_output_revolution);
}

std::optional<double> CanMotor::readPosition()
{
    bus_.collectPendingFeedback();
    const auto state = bus_.latestState(node_id_);
    if (!state || !state->position_counts)
    {
        return std::nullopt;
    }
    return positionCountsToRadians(
        state->position_counts->value,
        encoder_.counts_per_output_revolution);
}

std::optional<CspFeedback> CanMotor::queryCspStatus()
{
    return bus_.queryCsp(node_id_, std::chrono::milliseconds{50});
}

std::optional<DriverPositionLimits> CanMotor::queryPositionLimits()
{
    const auto maximum = bus_.queryPositionLimit(
        node_id_,
        PositionLimitKind::Maximum,
        std::chrono::milliseconds{50});
    const auto minimum = bus_.queryPositionLimit(
        node_id_,
        PositionLimitKind::Minimum,
        std::chrono::milliseconds{50});
    if (!minimum || !maximum || *minimum > *maximum)
    {
        return std::nullopt;
    }

    return DriverPositionLimits{
        *minimum,
        *maximum,
        positionCountsToRadians(
            *minimum,
            encoder_.counts_per_output_revolution),
        positionCountsToRadians(
            *maximum,
            encoder_.counts_per_output_revolution)};
}

std::optional<DriverStatus> CanMotor::queryDriverStatus()
{
    const auto run_mode = bus_.queryRunMode(
        node_id_,
        std::chrono::milliseconds{50});
    const auto fault_bits = bus_.queryFaultBits(
        node_id_,
        std::chrono::milliseconds{50});
    if (!run_mode || !fault_bits)
    {
        return std::nullopt;
    }
    return DriverStatus{*run_mode, *fault_bits};
}

std::optional<MotorState> CanMotor::latestState()
{
    bus_.collectPendingFeedback();
    return bus_.latestState(node_id_);
}

std::optional<MotorState> CanMotor::latestFeedback()
{
    return latestState();
}

std::optional<double> CanMotor::readVelocity()
{
    const auto state = latestState();
    if (!state || !state->speed_raw)
    {
        return std::nullopt;
    }
    return speedRawToOutputRadiansPerSecond(
        state->speed_raw->value,
        encoder_.gear_ratio);
}

std::optional<double> CanMotor::readCurrentAmps()
{
    const auto state = latestState();
    if (!state || !state->current_milliamps)
    {
        return std::nullopt;
    }
    return static_cast<double>(state->current_milliamps->value) / 1000.0;
}

bool CanMotor::hasFreshCspFeedback(
    const std::chrono::milliseconds maximum_age)
{
    if (maximum_age.count() < 0)
    {
        throw std::invalid_argument(
            "maximum CSP feedback age must not be negative");
    }
    const auto state = latestState();
    if (!state || !state->last_csp_feedback_timestamp)
    {
        return false;
    }
    return std::chrono::steady_clock::now() -
               *state->last_csp_feedback_timestamp <= maximum_age;
}

void CanMotor::commandPositionCsp(const double position_rad)
{
    const auto target_counts = radiansToPositionCounts(
        position_rad,
        encoder_.counts_per_output_revolution);
    bus_.send(encodePositionCsp(node_id_, target_counts));
}

void CanMotor::requestStopMode()
{
    bus_.send(encodeStopModeRequest(node_id_));
}

std::uint16_t CanMotor::nodeId() const noexcept
{
    return node_id_;
}

} // namespace robot::ti5
