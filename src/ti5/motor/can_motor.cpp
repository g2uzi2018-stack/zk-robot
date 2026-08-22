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

    bus_.registerNode(node_id_);
}

std::optional<double> CanMotor::queryPosition()
{
    const auto feedback = bus_.queryPosition(
        node_id_,
        std::chrono::milliseconds{50});
    if (!feedback || !feedback->position_counts)
    {
        return std::nullopt;
    }
    return positionCountsToRadians(
        *feedback->position_counts,
        encoder_.counts_per_output_revolution);
}

std::optional<double> CanMotor::readPosition()
{
    bus_.collectPendingFeedback();
    const auto feedback = bus_.latestFeedback(node_id_);
    if (!feedback || !feedback->position_counts)
    {
        return std::nullopt;
    }
    return positionCountsToRadians(
        *feedback->position_counts,
        encoder_.counts_per_output_revolution);
}

std::optional<MotorFeedback> CanMotor::queryCspStatus()
{
    return bus_.queryCsp(node_id_, std::chrono::milliseconds{50});
}

std::optional<MotorFeedback> CanMotor::latestFeedback()
{
    bus_.collectPendingFeedback();
    return bus_.latestFeedback(node_id_);
}

void CanMotor::commandPositionCsp(const double position_rad)
{
    const auto target_counts = radiansToPositionCounts(
        position_rad,
        encoder_.counts_per_output_revolution);
    bus_.send(encodePositionCsp(node_id_, target_counts));
}

std::uint16_t CanMotor::nodeId() const noexcept
{
    return node_id_;
}

} // namespace robot::ti5
