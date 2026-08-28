#include "ti5/hand/hand.hpp"

#include <stdexcept>
#include <utility>

namespace robot::ti5
{

hand::HandSideConfig Hand::validatedConfig(
    const HandSide side,
    const hand::HandSideConfig &config)
{
    const std::string expected_name =
        side == HandSide::Left ? "left_hand" : "right_hand";
    if (config.name != expected_name)
    {
        throw std::invalid_argument(
            "TI5 Hand side does not match hand config name");
    }
    if (config.protocol != "aoyi_hand" ||
        config.controller_node_id == 0)
    {
        throw std::invalid_argument(
            "TI5 Hand requires a valid aoyi_hand config");
    }
    return config;
}

Hand::Hand(
    const HandSide side,
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

Hand::Hand(
    const HandSide side,
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

HandSide Hand::side() const noexcept
{
    return side_;
}

const std::string &Hand::name() const noexcept
{
    return config_.name;
}

std::uint8_t Hand::controllerNodeId() const noexcept
{
    return config_.controller_node_id;
}

bool Hand::controlAllowed() const noexcept
{
    return config_.protocol_verified && config_.control_enabled;
}

std::optional<HandState> Hand::readState()
{
    const auto status = channel_.queryStatus();
    if (!status)
    {
        return std::nullopt;
    }
    return HandState{
        status->positions,
        status->forces,
        std::chrono::steady_clock::now()};
}

void Hand::commandPositionsRaw(
    const PositionValues &positions,
    const SpeedValues &speeds)
{
    if (!controlAllowed())
    {
        throw std::logic_error(
            "TI5 Hand control is disabled until protocol_verified and control_enabled are both true");
    }
    channel_.commandPositions(positions, speeds);
}

} // namespace robot::ti5
