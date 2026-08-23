#include "ti5/can/can_discovery.hpp"

#include "can/socket_can.hpp"
#include "common/logger.hpp"
#include "ti5/can/can_protocol.hpp"

#include <algorithm>
#include <chrono>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace
{

using robot::ti5::DiscoveryOptions;
using robot::ti5::InterfaceDiscoveryResult;

std::string formatNodeIds(const std::vector<std::uint16_t> &node_ids)
{
    if (node_ids.empty())
    {
        return "<none>";
    }

    std::ostringstream output;
    for (std::size_t index = 0; index < node_ids.size(); ++index)
    {
        if (index != 0)
        {
            output << ',';
        }
        output << node_ids[index];
    }
    return output.str();
}

std::vector<std::uint16_t> sortedNodeIds(std::vector<std::uint16_t> node_ids)
{
    std::sort(node_ids.begin(), node_ids.end());
    node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
    return node_ids;
}

bool containsNodeId(const std::vector<std::uint16_t> &node_ids,
                    const std::uint16_t node_id)
{
    return std::binary_search(node_ids.begin(), node_ids.end(), node_id);
}

std::vector<std::uint16_t> intersection(
    const std::vector<std::uint16_t> &expected_node_ids,
    const std::vector<std::uint16_t> &confirmed_node_ids)
{
    std::vector<std::uint16_t> matched;
    for (const auto node_id : expected_node_ids)
    {
        if (containsNodeId(confirmed_node_ids, node_id))
        {
            matched.push_back(node_id);
        }
    }
    return matched;
}

std::vector<std::uint16_t> missingNodeIds(
    const std::vector<std::uint16_t> &expected_node_ids,
    const std::vector<std::uint16_t> &confirmed_node_ids)
{
    std::vector<std::uint16_t> missing;
    for (const auto node_id : expected_node_ids)
    {
        if (!containsNodeId(confirmed_node_ids, node_id))
        {
            missing.push_back(node_id);
        }
    }
    return missing;
}

InterfaceDiscoveryResult scanInterface(
    const std::string &interface_name,
    const std::vector<std::uint16_t> &all_expected_node_ids,
    const DiscoveryOptions &options)
{
    InterfaceDiscoveryResult result;
    result.interface_name = interface_name;

    try
    {
        robot::can::SocketCan socket(interface_name);
        result.opened = true;

        // 清空打开前已经排队的帧，避免把旧响应计入本轮确认。
        while (socket.receive(std::chrono::milliseconds{0}))
        {
        }

        std::map<std::uint16_t, std::size_t> consecutive_confirmation_counts;
        for (const auto node_id : all_expected_node_ids)
        {
            consecutive_confirmation_counts.emplace(node_id, 0);
        }

        for (std::size_t attempt = 1; attempt <= options.max_attempts; ++attempt)
        {
            if (all_expected_node_ids.empty())
            {
                break;
            }

            robot::common::logger()->debug(
                "Scanning candidate {} attempt {}/{} for node IDs {}",
                interface_name,
                attempt,
                options.max_attempts,
                formatNodeIds(all_expected_node_ids));

            // 每轮先发送全部只读 0x08 查询，再在一个窗口中收集响应。
            for (const auto node_id : all_expected_node_ids)
            {
                socket.send(robot::ti5::encodePositionQuery(node_id));
            }

            std::set<std::uint16_t> responded_this_attempt;
            const auto deadline = std::chrono::steady_clock::now() + options.response_timeout;
            bool timed_out = false;

            while (std::chrono::steady_clock::now() < deadline)
            {
                const auto now = std::chrono::steady_clock::now();
                auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
                if (remaining.count() <= 0)
                {
                    remaining = std::chrono::milliseconds{1};
                }

                const auto frame = socket.receive(remaining);
                if (!frame)
                {
                    timed_out = true;
                    break;
                }
                if (!containsNodeId(all_expected_node_ids, frame->id))
                {
                    continue;
                }
                if (robot::ti5::isPositionQueryResponse(*frame, frame->id))
                {
                    // 同一轮中的重复响应只记一次。
                    responded_this_attempt.insert(frame->id);
                    if (responded_this_attempt.size() == all_expected_node_ids.size())
                    {
                        break;
                    }
                }
            }

            for (const auto node_id : all_expected_node_ids)
            {
                auto &confirmation_count = consecutive_confirmation_counts[node_id];
                if (responded_this_attempt.find(node_id) != responded_this_attempt.end())
                {
                    if (confirmation_count < options.confirmations_required)
                    {
                        ++confirmation_count;
                    }
                }
                else
                {
                    confirmation_count = 0;
                }
            }

            std::vector<std::uint16_t> confirmed_node_ids;
            for (const auto &[node_id, count] : consecutive_confirmation_counts)
            {
                if (count >= options.confirmations_required)
                {
                    confirmed_node_ids.push_back(node_id);
                }
            }

            std::vector<std::uint16_t> missing_node_ids;
            for (const auto node_id : all_expected_node_ids)
            {
                if (responded_this_attempt.find(node_id) == responded_this_attempt.end())
                {
                    missing_node_ids.push_back(node_id);
                }
            }

            robot::common::logger()->debug(
                "CAN interface {} scan round {}/{} received {} valid responses; confirmed nodes: {}",
                interface_name,
                attempt,
                options.max_attempts,
                responded_this_attempt.size(),
                formatNodeIds(confirmed_node_ids));
            if (timed_out)
            {
                robot::common::logger()->debug(
                    "CAN interface {} scan round {}/{} timed out after {} ms",
                    interface_name,
                    attempt,
                    options.max_attempts,
                    options.response_timeout.count());
            }
            robot::common::logger()->debug(
                "CAN interface {} scan round {}/{} missing valid responses for {}",
                interface_name,
                attempt,
                options.max_attempts,
                formatNodeIds(missing_node_ids));
        }

        for (const auto &[node_id, count] : consecutive_confirmation_counts)
        {
            if (count >= options.confirmations_required)
            {
                result.confirmed_node_ids.push_back(node_id);
            }
        }
        robot::common::logger()->info(
            "CAN interface {} final confirmed nodes: {}",
            interface_name,
            formatNodeIds(result.confirmed_node_ids));
    }
    catch (const std::exception &error)
    {
        result.error = error.what();
        robot::common::logger()->error(
            "CAN interface {} scan failed: {}",
            interface_name,
            error.what());
    }
    return result;
}

std::size_t countMatchedNodes(
    const robot::ti5::LogicalCanBus &bus,
    const InterfaceDiscoveryResult &interface_result)
{
    return intersection(bus.expected_node_ids,
                        interface_result.confirmed_node_ids).size();
}

} // namespace

namespace robot::ti5
{

DiscoveryResult CanDiscovery::discover(
    const std::vector<LogicalCanBus> &logical_buses,
    const DiscoveryOptions &options,
    const std::vector<std::string> &candidate_interfaces) const
{
    if (logical_buses.empty())
    {
        throw std::invalid_argument(
            "TI5 Discovery requires at least one logical CAN bus");
    }
    if (candidate_interfaces.empty())
    {
        throw std::invalid_argument(
            "TI5 Discovery requires at least one prepared candidate interface");
    }
    if (!options.require_interface_up)
    {
        throw std::invalid_argument(
            "TI5 Discovery requires prepared interfaces to be UP");
    }
    if (options.allow_partial_bus)
    {
        throw std::invalid_argument(
            "TI5 Discovery requires complete logical buses");
    }
    if (!options.require_unique_bus_match)
    {
        throw std::invalid_argument(
            "TI5 Discovery requires unique logical bus matches");
    }
    if (options.response_timeout.count() <= 0 ||
        options.confirmations_required == 0 ||
        options.max_attempts == 0 ||
        options.max_attempts < options.confirmations_required)
    {
        throw std::invalid_argument(
            "Invalid TI5 Discovery retry or timeout settings");
    }

    std::vector<std::uint16_t> all_expected_node_ids;
    std::unordered_map<std::uint16_t, std::string> node_owners;
    std::set<std::string> bus_names;
    for (const auto &bus : logical_buses)
    {
        if (bus.name.empty() || bus.expected_node_ids.empty())
        {
            throw std::invalid_argument(
                "TI5 logical CAN bus must have a name and expected node IDs");
        }
        if (!bus_names.insert(bus.name).second)
        {
            throw std::invalid_argument(
                "Duplicate TI5 logical CAN bus name: " + bus.name);
        }
        for (const auto node_id : bus.expected_node_ids)
        {
            if (node_id == 0 || node_id > 0x7FF)
            {
                throw std::invalid_argument(
                    "TI5 logical CAN bus contains an invalid node ID");
            }
            const auto [owner, inserted] = node_owners.emplace(node_id, bus.name);
            static_cast<void>(owner);
            if (!inserted)
            {
                throw std::invalid_argument(
                    "Node ID " + std::to_string(node_id) +
                    " belongs to multiple TI5 logical CAN buses");
            }
            all_expected_node_ids.push_back(node_id);
        }
    }
    all_expected_node_ids = sortedNodeIds(std::move(all_expected_node_ids));

    DiscoveryResult result;
    std::set<std::string> unique_candidates;
    for (const auto &interface_name : candidate_interfaces)
    {
        if (interface_name.empty() || !unique_candidates.insert(interface_name).second)
        {
            continue;
        }

        robot::common::logger()->info(
            "Scanning prepared body CAN interface {}",
            interface_name);
        result.interfaces.push_back(
            scanInterface(interface_name, all_expected_node_ids, options));
    }
    if (result.interfaces.empty())
    {
        throw std::invalid_argument(
            "TI5 Discovery candidate interface list contains no valid names");
    }

    // Interface indices and logical-bus indices are different domains.  In
    // particular, a filtered discovery may have three logical buses while the
    // matching device is candidate interface index 3.  Using
    // logical_buses.size() as the sentinel would then discard that valid
    // mapping.
    const std::size_t unassigned_interface = result.interfaces.size();
    std::vector<std::size_t> assigned_interface(
        logical_buses.size(), unassigned_interface);
    std::vector<bool> bus_conflict(logical_buses.size(), false);
    bool unique_match_failure = false;

    for (std::size_t interface_index = 0;
         interface_index < result.interfaces.size();
         ++interface_index)
    {
        const auto &interface_result = result.interfaces[interface_index];
        std::vector<std::size_t> candidates;
        for (std::size_t bus_index = 0;
             bus_index < logical_buses.size();
             ++bus_index)
        {
            const auto &bus = logical_buses[bus_index];
            if (countMatchedNodes(bus, interface_result) == bus.expected_node_ids.size())
            {
                candidates.push_back(bus_index);
            }
        }

        if (candidates.empty())
        {
            robot::common::logger()->debug(
                "CAN interface {} did not match any complete logical bus; confirmed nodes: {}",
                interface_result.interface_name,
                formatNodeIds(interface_result.confirmed_node_ids));
            continue;
        }
        if (candidates.size() > 1)
        {
            unique_match_failure = true;
            std::ostringstream names;
            for (std::size_t index = 0; index < candidates.size(); ++index)
            {
                if (index != 0)
                {
                    names << ',';
                }
                names << logical_buses[candidates[index]].name;
                bus_conflict[candidates[index]] = true;
            }
            robot::common::logger()->error(
                "CAN interface {} simultaneously matches logical buses {}; refusing ambiguous mapping",
                interface_result.interface_name,
                names.str());
            continue;
        }

        const auto bus_index = candidates.front();
        if (assigned_interface[bus_index] != unassigned_interface)
        {
            unique_match_failure = true;
            bus_conflict[bus_index] = true;
            robot::common::logger()->error(
                "Logical bus {} matches both {} and {}; refusing duplicate mapping",
                logical_buses[bus_index].name,
                result.interfaces[assigned_interface[bus_index]].interface_name,
                interface_result.interface_name);
            continue;
        }
        assigned_interface[bus_index] = interface_index;
    }

    for (std::size_t bus_index = 0; bus_index < logical_buses.size(); ++bus_index)
    {
        LogicalBusDiscoveryResult bus_result;
        bus_result.bus_name = logical_buses[bus_index].name;

        if (assigned_interface[bus_index] != unassigned_interface &&
            !bus_conflict[bus_index])
        {
            const auto &interface_result = result.interfaces[assigned_interface[bus_index]];
            bus_result.complete = true;
            bus_result.interface_name = interface_result.interface_name;
            bus_result.matched_node_ids = logical_buses[bus_index].expected_node_ids;
            result.logical_buses.push_back(std::move(bus_result));
            continue;
        }

        const InterfaceDiscoveryResult *best_interface = nullptr;
        std::vector<std::uint16_t> best_matched_node_ids;
        for (const auto &interface_result : result.interfaces)
        {
            const auto matched_node_ids = intersection(
                logical_buses[bus_index].expected_node_ids,
                interface_result.confirmed_node_ids);
            if (matched_node_ids.size() > best_matched_node_ids.size())
            {
                best_interface = &interface_result;
                best_matched_node_ids = matched_node_ids;
            }
        }

        bus_result.matched_node_ids = std::move(best_matched_node_ids);
        bus_result.missing_node_ids = missingNodeIds(
            logical_buses[bus_index].expected_node_ids,
            bus_result.matched_node_ids);
        const bool bus_required = logical_buses[bus_index].required ||
                                  !options.allow_partial_bus;

        if (best_interface != nullptr && !bus_conflict[bus_index])
        {
            bus_result.interface_name = best_interface->interface_name;
            robot::common::logger()->warn(
                "Logical bus {} is incomplete; best interface {} matched nodes {} and is missing nodes {}",
                bus_result.bus_name,
                best_interface->interface_name,
                formatNodeIds(bus_result.matched_node_ids),
                formatNodeIds(bus_result.missing_node_ids));
        }
        else if (best_interface != nullptr)
        {
            bus_result.interface_name = best_interface->interface_name;
        }

        if (bus_required && !bus_conflict[bus_index])
        {
            unique_match_failure = true;
            robot::common::logger()->error(
                "Required logical bus {} has no complete mapping; matched node IDs [{}], missing node IDs [{}]",
                bus_result.bus_name,
                formatNodeIds(bus_result.matched_node_ids),
                formatNodeIds(bus_result.missing_node_ids));
        }
        else if (best_interface == nullptr && !bus_conflict[bus_index])
        {
            robot::common::logger()->warn(
                "Logical bus {} has no responding CAN interface; missing nodes {}",
                bus_result.bus_name,
                formatNodeIds(bus_result.missing_node_ids));
        }
        result.logical_buses.push_back(std::move(bus_result));
    }

    result.success = !unique_match_failure;
    for (const auto &bus_result : result.logical_buses)
    {
        if (bus_result.complete && bus_result.interface_name)
        {
            robot::common::logger()->info(
                "Final logical bus mapping: {} -> {}",
                bus_result.bus_name,
                *bus_result.interface_name);
        }
    }

    if (!result.success)
    {
        robot::common::logger()->error(
            "TI5 CAN Discovery failed; at least one logical bus is incomplete or ambiguous");
    }
    return result;
}

} // namespace robot::ti5
