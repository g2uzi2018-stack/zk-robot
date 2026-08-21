#include "ti5/discovery.hpp"

#include "can/socket_can.hpp"
#include "common/logger.hpp"
#include "ti5/can/can_protocol.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <map>
#include <net/if.h>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <unordered_map>
#include <utility>

namespace
{
    using robot::ti5::DiscoveryOptions;
    using robot::ti5::InterfaceDiscoveryResult;

    struct InterfaceCandidate
    {
        std::string name;
        unsigned long long number{0};
    };

    std::string formatNodeIds(const std::vector<std::uint16_t> &node_ids)
    {
        if (node_ids.empty())
        {
            return "<none>";
        }

        std::ostringstream output;
        for (std::size_t i = 0; i < node_ids.size(); ++i)
        {
            if (i != 0)
            {
                output << ',';
            }
            output << node_ids[i];
        }
        return output.str();
    }

    std::vector<std::uint16_t> sortedNodeIds(std::vector<std::uint16_t> node_ids)
    {
        std::sort(node_ids.begin(), node_ids.end());
        node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
        return node_ids;
    }

    bool containsNodeId(const std::vector<std::uint16_t> &node_ids, const std::uint16_t node_id)
    {
        return std::binary_search(node_ids.begin(), node_ids.end(), node_id);
    }

    std::vector<std::uint16_t> intersection(const std::vector<std::uint16_t> &expected_node_ids,
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

    std::vector<std::uint16_t> missingNodeIds(const std::vector<std::uint16_t> &expected_node_ids,
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

    bool isInterfaceUp(const std::string &interface_name)
    {
        const int socket_fd = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
        if (socket_fd < 0)
        {
            robot::common::logger()->error("Cannot inspect CAN interface {} state: {}",
                                           interface_name,
                                           std::strerror(errno));
            return false;
        }

        ifreq request{};
        std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
        const int ioctl_result = ::ioctl(socket_fd, SIOCGIFFLAGS, &request);
        const int saved_errno = errno;
        ::close(socket_fd);

        if (ioctl_result < 0)
        {
            robot::common::logger()->error("Cannot inspect CAN interface {} state: {}",
                                           interface_name,
                                           std::strerror(saved_errno));
            return false;
        }
        return (request.ifr_flags & IFF_UP) != 0;
    }

    std::vector<InterfaceCandidate> enumerateUpCanInterfaces(const DiscoveryOptions &options)
    {
        const std::regex interface_pattern(options.interface_regex);
        struct if_nameindex *interfaces = ::if_nameindex();
        if (interfaces == nullptr)
        {
            throw std::system_error(errno, std::generic_category(), "Enumerate network interfaces");
        }

        std::vector<InterfaceCandidate> candidates;
        for (struct if_nameindex *entry = interfaces; entry->if_index != 0; ++entry)
        {
            if (entry->if_name == nullptr)
            {
                continue;
            }

            const std::string interface_name{entry->if_name};
            if (!std::regex_match(interface_name, interface_pattern))
            {
                continue;
            }

            unsigned long long interface_number{};
            try
            {
                // 配置正则严格为 ^can[0-9]+$，因此这里无需修改正则增加捕获组。
                interface_number = std::stoull(interface_name.substr(3));
            }
            catch (const std::exception &error)
            {
                robot::common::logger()->debug("Ignoring CAN interface {} with invalid numeric suffix: {}",
                                               interface_name,
                                               error.what());
                continue;
            }

            if (options.require_interface_up && !isInterfaceUp(interface_name))
            {
                robot::common::logger()->debug("Ignoring CAN interface {} because it is not UP", interface_name);
                continue;
            }

            candidates.push_back({interface_name, interface_number});
        }
        ::if_freenameindex(interfaces);

        std::sort(candidates.begin(), candidates.end(), [](const auto &left, const auto &right) {
            if (left.number != right.number)
            {
                return left.number < right.number;
            }
            return left.name < right.name;
        });
        return candidates;
    }

    InterfaceDiscoveryResult scanInterface(const std::string &interface_name,
                                            const std::vector<std::uint16_t> &all_expected_node_ids,
                                            const DiscoveryOptions &options)
    {
        InterfaceDiscoveryResult result;
        result.interface_name = interface_name;

        try
        {
            robot::can::SocketCan socket(interface_name);
            result.opened = true;

            // 新建 SocketCan 后先清空当前接收队列，避免把打开前排队的帧当成确认。
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
                // 每一轮都查询全部节点，才能让本轮没有有效响应的节点连续确认计数清零。
                const auto &nodes_to_query = all_expected_node_ids;

                if (nodes_to_query.empty())
                {
                    break;
                }

                robot::common::logger()->debug("Scanning {} attempt {}/{} for node IDs {}",
                                              interface_name,
                                              attempt,
                                              options.max_attempts,
                                              formatNodeIds(nodes_to_query));

                // 一个 attempt 先批量发送请求，再在单一时间窗口内集中接收响应。
                for (const auto node_id : nodes_to_query)
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

                    if (!containsNodeId(nodes_to_query, frame->id))
                    {
                        continue;
                    }

                    if (robot::ti5::isPositionQueryResponse(*frame, frame->id))
                    {
                        // 同一个 attempt 内的重复帧只记一次确认，避免单次响应洪泛满足多次确认。
                        responded_this_attempt.insert(frame->id);
                        if (responded_this_attempt.size() == nodes_to_query.size())
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
                for (const auto node_id : nodes_to_query)
                {
                    if (responded_this_attempt.find(node_id) == responded_this_attempt.end())
                    {
                        missing_node_ids.push_back(node_id);
                    }
                }

                robot::common::logger()->debug("CAN interface {} scan round {}/{} received {} valid responses; consecutive confirmed nodes: {}",
                                              interface_name,
                                              attempt,
                                              options.max_attempts,
                                              responded_this_attempt.size(),
                                              formatNodeIds(confirmed_node_ids));

                if (timed_out)
                {
                    robot::common::logger()->debug("CAN interface {} scan round {}/{} timed out after {} ms",
                                                   interface_name,
                                                   attempt,
                                                   options.max_attempts,
                                                   options.response_timeout.count());
                }
                robot::common::logger()->debug("CAN interface {} scan round {}/{} missing valid responses for {}",
                                               interface_name,
                                               attempt,
                                               options.max_attempts,
                                               formatNodeIds(missing_node_ids));

                if (attempt < options.max_attempts && !missing_node_ids.empty())
                {
                    robot::common::logger()->debug("CAN interface {} retrying scan round {}/{} after missing responses for {}",
                                                   interface_name,
                                                   attempt + 1,
                                                   options.max_attempts,
                                                   formatNodeIds(missing_node_ids));
                }
            }

            for (const auto &[node_id, count] : consecutive_confirmation_counts)
            {
                if (count >= options.confirmations_required)
                {
                    result.confirmed_node_ids.push_back(node_id);
                }
            }
            robot::common::logger()->info("CAN interface {} final confirmed nodes: {}",
                                         interface_name,
                                         formatNodeIds(result.confirmed_node_ids));
        }
        catch (const std::exception &error)
        {
            result.error = error.what();
            robot::common::logger()->error("CAN interface {} scan failed: {}", interface_name, error.what());
        }
        return result;
    }

    std::size_t countMatchedNodes(const robot::ti5::LogicalCanBus &bus,
                                  const InterfaceDiscoveryResult &interface_result)
    {
        return intersection(bus.expected_node_ids, interface_result.confirmed_node_ids).size();
    }
}

namespace robot::ti5
{
    DiscoveryResult CanDiscovery::discover(const std::vector<LogicalCanBus> &logical_buses,
                                           const DiscoveryOptions &options) const
    {
        if (logical_buses.empty())
        {
            throw std::invalid_argument("TI5 Discovery requires at least one logical CAN bus");
        }
        if (options.interface_regex != "^can[0-9]+$")
        {
            throw std::invalid_argument("TI5 Discovery only supports interface regex '^can[0-9]+$'");
        }
        if (!options.require_interface_up)
        {
            throw std::invalid_argument("TI5 Discovery requires interfaces to be UP");
        }
        if (options.allow_partial_bus)
        {
            throw std::invalid_argument("TI5 Discovery requires complete logical buses");
        }
        if (!options.require_unique_bus_match)
        {
            throw std::invalid_argument("TI5 Discovery requires unique logical bus matches");
        }
        if (options.response_timeout.count() <= 0 || options.confirmations_required == 0 || options.max_attempts == 0 ||
            options.max_attempts < options.confirmations_required)
        {
            throw std::invalid_argument("Invalid TI5 Discovery retry or timeout settings");
        }

        std::vector<std::uint16_t> all_expected_node_ids;
        std::unordered_map<std::uint16_t, std::string> node_owners;
        std::set<std::string> bus_names;
        for (const auto &bus : logical_buses)
        {
            if (bus.name.empty() || bus.expected_node_ids.empty())
            {
                throw std::invalid_argument("TI5 logical CAN bus must have a name and expected node IDs");
            }
            if (!bus_names.insert(bus.name).second)
            {
                throw std::invalid_argument("Duplicate TI5 logical CAN bus name: " + bus.name);
            }
            for (const auto node_id : bus.expected_node_ids)
            {
                if (node_id == 0 || node_id > 0x7FF)
                {
                    throw std::invalid_argument("TI5 logical CAN bus contains an invalid node ID");
                }
                const auto [owner, inserted] = node_owners.emplace(node_id, bus.name);
                if (!inserted)
                {
                    throw std::invalid_argument("Node ID " + std::to_string(node_id) +
                                                " belongs to multiple TI5 logical CAN buses");
                }
                all_expected_node_ids.push_back(node_id);
            }
        }
        all_expected_node_ids = sortedNodeIds(std::move(all_expected_node_ids));

        const auto interfaces = enumerateUpCanInterfaces(options);
        DiscoveryResult result;
        for (const auto &interface : interfaces)
        {
            robot::common::logger()->info("Found UP CAN interface {}", interface.name);
            result.interfaces.push_back(scanInterface(interface.name, all_expected_node_ids, options));
        }

        if (result.interfaces.empty())
        {
            robot::common::logger()->debug("No UP CAN interface matching {} was found", options.interface_regex);
        }

        std::vector<std::size_t> assigned_interface(logical_buses.size(), logical_buses.size());
        std::vector<bool> bus_conflict(logical_buses.size(), false);
        bool unique_match_failure = false;

        for (std::size_t interface_index = 0; interface_index < result.interfaces.size(); ++interface_index)
        {
            const auto &interface_result = result.interfaces[interface_index];
            std::vector<std::size_t> candidates;
            for (std::size_t bus_index = 0; bus_index < logical_buses.size(); ++bus_index)
            {
                const auto &bus = logical_buses[bus_index];
                if (countMatchedNodes(bus, interface_result) == bus.expected_node_ids.size())
                {
                    candidates.push_back(bus_index);
                }
            }

            if (candidates.empty())
            {
                robot::common::logger()->debug("CAN interface {} did not match any complete logical bus; confirmed nodes: {}",
                                               interface_result.interface_name,
                                               formatNodeIds(interface_result.confirmed_node_ids));
                continue;
            }

            if (candidates.size() > 1)
            {
                unique_match_failure = true;
                std::ostringstream bus_names_output;
                for (std::size_t i = 0; i < candidates.size(); ++i)
                {
                    if (i != 0)
                    {
                        bus_names_output << ',';
                    }
                    bus_names_output << logical_buses[candidates[i]].name;
                    bus_conflict[candidates[i]] = true;
                }
                robot::common::logger()->error("CAN interface {} simultaneously matches logical buses {}; refusing ambiguous mapping",
                                               interface_result.interface_name,
                                               bus_names_output.str());
                continue;
            }

            const auto bus_index = candidates.front();
            if (assigned_interface[bus_index] != logical_buses.size())
            {
                unique_match_failure = true;
                bus_conflict[bus_index] = true;
                robot::common::logger()->error("Logical bus {} matches both {} and {}; refusing duplicate mapping",
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

            if (assigned_interface[bus_index] != logical_buses.size() && !bus_conflict[bus_index])
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
                const auto matched_node_ids = intersection(logical_buses[bus_index].expected_node_ids,
                                                           interface_result.confirmed_node_ids);
                if (matched_node_ids.size() > best_matched_node_ids.size())
                {
                    best_interface = &interface_result;
                    best_matched_node_ids = matched_node_ids;
                }
            }

            bus_result.matched_node_ids = std::move(best_matched_node_ids);
            bus_result.missing_node_ids = missingNodeIds(logical_buses[bus_index].expected_node_ids,
                                                          bus_result.matched_node_ids);
            const bool bus_required = logical_buses[bus_index].required || !options.allow_partial_bus;
            if (best_interface != nullptr && !bus_conflict[bus_index])
            {
                bus_result.interface_name = best_interface->interface_name;
                robot::common::logger()->warn("Logical bus {} is incomplete on every interface; best interface {} matched nodes {} and is missing nodes {}",
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
                robot::common::logger()->error("Required logical bus {} has no complete mapping; matched node IDs [{}], missing node IDs [{}]",
                                               bus_result.bus_name,
                                               formatNodeIds(bus_result.matched_node_ids),
                                               formatNodeIds(bus_result.missing_node_ids));
            }
            else if (best_interface == nullptr && !bus_conflict[bus_index])
            {
                robot::common::logger()->warn("Logical bus {} has no responding CAN interface; missing nodes {}",
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
                robot::common::logger()->info("Final logical bus mapping: {} -> {}",
                                             bus_result.bus_name,
                                             *bus_result.interface_name);
            }
        }

        if (!result.success)
        {
            robot::common::logger()->error("TI5 CAN Discovery failed; at least one logical bus is incomplete or ambiguous");
            return result;
        }
        return result;
    }
}
