#include "ti5/gripper/gripper.hpp"

#include <stdexcept>
#include <utility>

namespace robot::ti5
{

hand::HandSideConfig Gripper::validatedConfig(
    const GripperSide side,
    const hand::HandSideConfig &config)
{
    const std::string expected_name =
        side == GripperSide::Left ? "left_hand" : "right_hand";
    if (config.name != expected_name)
    {
        throw std::invalid_argument(
            "TI5 Gripper side does not match hand config name");
    }
    if (config.protocol != "aoyi_hand" ||
        config.controller_node_id == 0)
    {
        throw std::invalid_argument(
            "TI5 Gripper requires a valid aoyi_hand config");
    }
    return config;
}

Gripper::Gripper(
    const GripperSide side,
    const hand::HandSideConfig &config,
    std::string interface_name,
    const std::chrono::milliseconds response_timeout)
    : side_(side),
      config_(validatedConfig(side, config)),
      channel_(
          std::move(interface_name),
          config_.controller_node_id,
          response_timeout)
{
}

Gripper::Gripper(
    const GripperSide side,
    const hand::HandSideConfig &config,
    std::unique_ptr<hand::HandTransport> transport,
    const std::chrono::milliseconds response_timeout)
    : side_(side),
      config_(validatedConfig(side, config)),
      channel_(
          std::move(transport),
          config_.controller_node_id,
          response_timeout)
{
}

GripperSide Gripper::side() const noexcept
{
    return side_;
}

const std::string &Gripper::name() const noexcept
{
    return config_.name;
}

std::uint8_t Gripper::controllerNodeId() const noexcept
{
    return config_.controller_node_id;
}

bool Gripper::controlAllowed() const noexcept
{
    return config_.protocol_verified && config_.control_enabled;
}

std::optional<GripperState> Gripper::readState()
{
    const auto status = channel_.queryStatus();
    if (!status)
    {
        return std::nullopt;
    }
    return GripperState{
        status->positions,
        status->forces,
        std::chrono::steady_clock::now()};
}

void Gripper::commandPositionsRaw(
    const PositionValues &positions,
    const SpeedValues &speeds)
{
    if (!controlAllowed())
    {
        throw std::logic_error(
            "TI5 Gripper control is disabled until protocol_verified and control_enabled are both true");
    }
    channel_.commandPositions(positions, speeds);
}

} // namespace robot::ti5
