#include "ti5/joint/joint.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace robot::ti5
{
namespace
{

void requireFinite(const double value, const char *description)
{
    if (!std::isfinite(value))
    {
        throw std::invalid_argument(std::string{description} + " must be finite");
    }
}

} // namespace

Joint::Joint(const JointConfig &config, CanBus &bus)
    : name_(config.physical_joint.name),
      physical_name_(config.physical_joint.physical_name),
      bus_name_(config.physical_joint.bus),
      motor_position_limits_(config.motor_position_limits),
      coordinate_transform_(config.coordinate_transform),
      motor_(config.physical_joint.motor, bus)
{
    if (name_.empty() || physical_name_.empty() || bus_name_.empty())
    {
        throw std::invalid_argument("TI5 Joint identity fields must not be empty");
    }
    requireFinite(coordinate_transform_.direction, "joint direction");
    requireFinite(coordinate_transform_.offset_rad, "joint offset");
    if (coordinate_transform_.direction != 1.0 &&
        coordinate_transform_.direction != -1.0)
    {
        throw std::invalid_argument("TI5 Joint direction must be +1 or -1");
    }
    requireFinite(motor_position_limits_.minimum_rad, "motor minimum position");
    requireFinite(motor_position_limits_.maximum_rad, "motor maximum position");
    if (motor_position_limits_.minimum_rad >= motor_position_limits_.maximum_rad)
    {
        throw std::invalid_argument(
            "TI5 Joint motor position limits must satisfy minimum < maximum");
    }

    const double first = motorToJointPosition(
        motor_position_limits_.minimum_rad);
    const double second = motorToJointPosition(
        motor_position_limits_.maximum_rad);
    joint_position_limits_ = JointPositionLimits{
        std::min(first, second),
        std::max(first, second),
        motor_position_limits_.verified_on_robot};
}

const std::string &Joint::name() const noexcept
{
    return name_;
}

const std::string &Joint::physicalName() const noexcept
{
    return physical_name_;
}

const std::string &Joint::busName() const noexcept
{
    return bus_name_;
}

std::uint16_t Joint::nodeId() const noexcept
{
    return motor_.nodeId();
}

double Joint::motorToJointPosition(const double motor_position_rad) const
{
    requireFinite(motor_position_rad, "motor position");
    return (motor_position_rad - coordinate_transform_.offset_rad) /
           coordinate_transform_.direction;
}

double Joint::jointToMotorPosition(const double joint_position_rad) const
{
    requireFinite(joint_position_rad, "joint position");
    return joint_position_rad * coordinate_transform_.direction +
           coordinate_transform_.offset_rad;
}

const JointPositionLimits &Joint::motorPositionLimits() const noexcept
{
    return motor_position_limits_;
}

const JointPositionLimits &Joint::positionLimits() const noexcept
{
    return joint_position_limits_;
}

std::optional<DriverPositionLimits> Joint::refreshDriverPositionLimits()
{
    const auto limits = motor_.queryPositionLimits();
    if (!limits)
    {
        driver_position_limits_.reset();
        return std::nullopt;
    }
    if (!std::isfinite(limits->minimum_rad) ||
        !std::isfinite(limits->maximum_rad) ||
        limits->minimum_rad >= limits->maximum_rad)
    {
        driver_position_limits_.reset();
        return std::nullopt;
    }
    driver_position_limits_ = *limits;
    return driver_position_limits_;
}

std::optional<DriverPositionLimits> Joint::driverPositionLimits() const noexcept
{
    return driver_position_limits_;
}

std::optional<JointPositionLimits> Joint::positionCommandLimits() const
{
    if (!driver_position_limits_)
    {
        return std::nullopt;
    }
    const double motor_minimum = std::max(
        motor_position_limits_.minimum_rad,
        driver_position_limits_->minimum_rad);
    const double motor_maximum = std::min(
        motor_position_limits_.maximum_rad,
        driver_position_limits_->maximum_rad);
    if (motor_minimum > motor_maximum)
    {
        return std::nullopt;
    }
    const double first = motorToJointPosition(motor_minimum);
    const double second = motorToJointPosition(motor_maximum);
    return JointPositionLimits{
        std::min(first, second),
        std::max(first, second),
        motor_position_limits_.verified_on_robot};
}

void Joint::validatePositionCommand(const double joint_position_rad) const
{
    const double motor_position_rad = jointToMotorPosition(joint_position_rad);
    if (motor_position_rad < motor_position_limits_.minimum_rad ||
        motor_position_rad > motor_position_limits_.maximum_rad)
    {
        throw std::out_of_range(
            name_ + " target exceeds safety.yaml motor position limits");
    }
    if (!driver_position_limits_)
    {
        throw std::logic_error(
            name_ + " driver position limits have not been loaded");
    }
    if (motor_position_rad < driver_position_limits_->minimum_rad ||
        motor_position_rad > driver_position_limits_->maximum_rad)
    {
        throw std::out_of_range(
            name_ + " target exceeds driver position target limits");
    }
}

void Joint::commandPositionCsp(const double joint_position_rad)
{
    validatePositionCommand(joint_position_rad);
    motor_.commandPositionCsp(jointToMotorPosition(joint_position_rad));
}

std::optional<double> Joint::queryPosition()
{
    const auto motor_position = motor_.queryPosition();
    if (!motor_position)
    {
        return std::nullopt;
    }
    return motorToJointPosition(*motor_position);
}

std::optional<double> Joint::readPosition()
{
    const auto motor_position = motor_.readPosition();
    if (!motor_position)
    {
        return std::nullopt;
    }
    return motorToJointPosition(*motor_position);
}

std::optional<double> Joint::readVelocity()
{
    const auto motor_velocity = motor_.readVelocity();
    if (!motor_velocity)
    {
        return std::nullopt;
    }
    return *motor_velocity / coordinate_transform_.direction;
}

std::optional<double> Joint::readCurrentAmps()
{
    return motor_.readCurrentAmps();
}

std::optional<CspFeedback> Joint::queryMotorCspStatus()
{
    return motor_.queryCspStatus();
}

std::optional<DriverStatus> Joint::queryDriverStatus()
{
    return motor_.queryDriverStatus();
}

std::optional<MotorState> Joint::latestMotorState()
{
    return motor_.latestState();
}

bool Joint::hasFreshCspFeedback(
    const std::chrono::milliseconds maximum_age)
{
    return motor_.hasFreshCspFeedback(maximum_age);
}

} // namespace robot::ti5
