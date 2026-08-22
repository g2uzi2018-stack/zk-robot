#include "ti5/hand/hand_discovery.hpp"

#include "can/socket_can.hpp"
#include "common/logger.hpp"
#include "ti5/hand/aoyi_protocol.hpp"

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

using robot::ti5::hand::HandConfig;
using robot::ti5::hand::HandInterfaceDiscoveryResult;

std::vector<std::uint8_t> enabledHandIds(const HandConfig &config)
{
    std::vector<std::uint8_t> ids;
    if (config.left.discovery_enabled)
    {
        ids.push_back(config.left.controller_node_id);
    }
    if (config.right.discovery_enabled)
    {
        ids.push_back(config.right.controller_node_id);
    }
    return ids;
}

bool contains(
    const std::set<std::uint8_t> &values,
    const std::uint8_t value)
{
    return values.find(value) != values.end();
}

void sendStatusQuery(
    robot::can::SocketCan &socket,
    const std::uint8_t hand_id)
{
    const auto bytes = robot::ti5::hand::encodePacket(
        hand_id,
        robot::ti5::hand::kAoyiGetStatusCommand,
        {robot::ti5::hand::kAoyiGetStatusSubcommand});
    for (const auto &frame :
         robot::ti5::hand::fragmentPacket(hand_id, bytes))
    {
        socket.send(frame);
    }
}

HandInterfaceDiscoveryResult scanInterface(
    const HandConfig &config,
    const std::string &interface_name)
{
    HandInterfaceDiscoveryResult result;
    result.interface_name = interface_name;

    try
    {
        robot::can::SocketCan socket(interface_name);
        result.opened = true;

        while (socket.receive(std::chrono::milliseconds{0}))
        {
        }

        const auto ids = enabledHandIds(config);
        std::map<std::uint8_t, std::size_t> confirmations;
        for (const auto id : ids)
        {
            confirmations.emplace(id, 0);
        }

        for (std::size_t attempt = 1;
             attempt <= config.discovery.max_attempts;
             ++attempt)
        {
            std::map<
                std::uint8_t,
                std::unique_ptr<robot::ti5::hand::PacketReassembler>>
                reassemblers;
            for (const auto id : ids)
            {
                reassemblers.emplace(
                    id,
                    std::make_unique<robot::ti5::hand::PacketReassembler>(
                        id,
                        id,
                        config.discovery.response_timeout));
                sendStatusQuery(socket, id);
            }

            std::set<std::uint8_t> responded;
            const auto deadline =
                std::chrono::steady_clock::now() +
                config.discovery.response_timeout;
            while (std::chrono::steady_clock::now() < deadline)
            {
                const auto now = std::chrono::steady_clock::now();
                auto remaining =
                    std::chrono::duration_cast<std::chrono::milliseconds>(
                        deadline - now);
                if (remaining.count() <= 0)
                {
                    remaining = std::chrono::milliseconds{1};
                }

                const auto frame = socket.receive(remaining);
                if (!frame)
                {
                    break;
                }

                const auto it = reassemblers.find(
                    static_cast<std::uint8_t>(frame->id));
                if (it == reassemblers.end())
                {
                    continue;
                }

                const auto packet = it->second->push(*frame, now);
                if (packet &&
                    packet->hand_id == frame->id &&
                    packet->command ==
                        robot::ti5::hand::kAoyiGetStatusCommand)
                {
                    responded.insert(packet->hand_id);
                }

                if (responded.size() == ids.size())
                {
                    break;
                }
            }

            for (const auto id : ids)
            {
                auto &count = confirmations[id];
                if (contains(responded, id))
                {
                    ++count;
                }
                else
                {
                    count = 0;
                }
                if (count >= config.discovery.confirmations_required)
                {
                    result.confirmed_hand_ids.insert(id);
                }
            }

            if (result.confirmed_hand_ids.size() == ids.size())
            {
                break;
            }
        }

        robot::common::logger()->info(
            "Aoyi CAN interface {} confirmed {} hand controller(s)",
            interface_name,
            result.confirmed_hand_ids.size());
    }
    catch (const std::exception &error)
    {
        result.error = error.what();
        robot::common::logger()->error(
            "Aoyi CAN interface {} scan failed: {}",
            interface_name,
            error.what());
    }
    return result;
}

} // namespace

namespace robot::ti5::hand
{

HandDiscoveryResult HandDiscovery::resolve(
    const HandConfig &config,
    const std::vector<HandInterfaceDiscoveryResult> &scan_results) const
{
    HandDiscoveryResult result;
    result.interfaces = scan_results;

    const auto findMatches =
        [&scan_results](
            const std::uint8_t hand_id)
        {
            std::vector<std::string> matches;
            for (const auto &scan : scan_results)
            {
                if (scan.opened && contains(scan.confirmed_hand_ids, hand_id))
                {
                    matches.push_back(scan.interface_name);
                }
            }
            return matches;
        };

    const auto resolveSide =
        [&result, &findMatches](
            const std::string &side_name,
            const std::uint8_t hand_id)
        {
            const auto matches = findMatches(hand_id);
            if (matches.size() == 1)
            {
                return std::optional<std::string>{matches.front()};
            }
            if (matches.empty())
            {
                result.errors.push_back(
                    side_name + " HAND_ID " +
                    std::to_string(hand_id) +
                    " did not answer on any candidate interface");
            }
            else
            {
                result.errors.push_back(
                    side_name + " HAND_ID " +
                    std::to_string(hand_id) +
                    " answered on multiple interfaces");
            }
            return std::optional<std::string>{};
        };

    if (config.left.discovery_enabled)
    {
        result.left_interface = resolveSide(
            "left_hand",
            config.left.controller_node_id);
    }
    if (config.right.discovery_enabled)
    {
        result.right_interface = resolveSide(
            "right_hand",
            config.right.controller_node_id);
    }

    result.success = result.errors.empty();
    return result;
}

HandDiscoveryResult HandDiscovery::discover(
    const HandConfig &config,
    const std::vector<std::string> &candidate_interfaces) const
{
    if (candidate_interfaces.empty())
    {
        throw std::invalid_argument(
            "Aoyi Hand Discovery requires candidate interfaces");
    }
    if (config.discovery.response_timeout.count() <= 0 ||
        config.discovery.confirmations_required == 0 ||
        config.discovery.max_attempts == 0 ||
        config.discovery.max_attempts <
            config.discovery.confirmations_required)
    {
        throw std::invalid_argument(
            "Invalid Aoyi Hand Discovery timeout or retry settings");
    }

    std::set<std::string> unique_interfaces;
    std::vector<HandInterfaceDiscoveryResult> scans;
    for (const auto &interface_name : candidate_interfaces)
    {
        if (interface_name.empty() ||
            !unique_interfaces.insert(interface_name).second)
        {
            continue;
        }
        scans.push_back(scanInterface(config, interface_name));
    }
    if (scans.empty())
    {
        throw std::invalid_argument(
            "Aoyi Hand Discovery candidate list contains no names");
    }

    return resolve(config, scans);
}

} // namespace robot::ti5::hand
