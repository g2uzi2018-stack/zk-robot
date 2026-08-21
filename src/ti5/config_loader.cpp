#include "ti5/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace
{
    template <typename T>
    T requireScalar(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!value.IsScalar())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be scalar in " + context);
        }

        try
        {
            return value.as<T>();
        }
        catch (const YAML::Exception &error)
        {
            throw std::invalid_argument("Invalid '" + std::string(key) + "' in " + context + ": " + error.what());
        }
    }

    YAML::Node requireMap(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!value.IsMap())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be map in " + context);
        }
        return value;
    }

    YAML::Node requireSequence(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!value.IsSequence())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be sequence in " + context);
        }
        return value;
    }

    std::uint16_t parseNodeId(const YAML::Node &node, const std::string &context)
    {
        if (!node || !node.IsScalar())
        {
            throw std::invalid_argument("Node ID must be scalar in " + context);
        }

        int value{};
        try
        {
            value = node.as<int>();
        }
        catch (const YAML::Exception &error)
        {
            throw std::invalid_argument("Invalid node ID in " + context + ": " + error.what());
        }

        // T170C 使用标准 11-bit CAN ID；0 保留为有效 CAN ID，但不是当前电机节点配置。
        if (value <= 0 || value > 0x7FF)
        {
            throw std::invalid_argument("Node ID must be in range 1..2047 in " + context);
        }
        return static_cast<std::uint16_t>(value);
    }

    void validateDiscoverySettings(const robot::ti5::DiscoveryOptions &options,
                                   const std::string &context)
    {
        if (options.interface_regex != "^can[0-9]+$")
        {
            throw std::invalid_argument("socketcan.interface_regex must be '^can[0-9]+$' in " + context);
        }
        if (!options.require_interface_up)
        {
            throw std::invalid_argument("socketcan.require_interface_up must be true in " + context);
        }
        if (options.response_timeout.count() <= 0)
        {
            throw std::invalid_argument("discovery.response_timeout_ms must be positive in " + context);
        }
        if (options.confirmations_required == 0)
        {
            throw std::invalid_argument("discovery.confirmations_required must be positive in " + context);
        }
        if (options.max_attempts == 0 || options.max_attempts < options.confirmations_required)
        {
            throw std::invalid_argument(
                "discovery.max_attempts must be positive and at least confirmations_required in " + context);
        }
    }
}

namespace robot::ti5
{
    Ti5RobotConfig loadRobotConfig(const std::filesystem::path &config_path)
    {
        const auto root = YAML::LoadFile(config_path.string());
        // robot.yaml 将机器人元数据放在 robot 下，将逻辑 CAN 分区放在顶层。
        requireMap(root, "robot", config_path.string());
        const auto buses = requireSequence(root, "can_buses", config_path.string());

        Ti5RobotConfig result;
        std::unordered_set<std::string> bus_names;
        std::unordered_map<std::uint16_t, std::string> node_owners;

        for (std::size_t i = 0; i < buses.size(); ++i)
        {
            const auto context = config_path.string() + ".can_buses[" + std::to_string(i) + "]";
            if (!buses[i].IsMap())
            {
                throw std::invalid_argument("Logical CAN bus must be map in " + context);
            }

            LogicalCanBus bus;
            bus.name = requireScalar<std::string>(buses[i], "name", context);
            bus.protocol = requireScalar<std::string>(buses[i], "protocol", context);
            bus.required = requireScalar<bool>(buses[i], "required", context);

            if (bus.name.empty())
            {
                throw std::invalid_argument("Logical CAN bus name must not be empty in " + context);
            }
            if (!bus_names.insert(bus.name).second)
            {
                throw std::invalid_argument("Duplicate logical CAN bus name '" + bus.name + "'");
            }

            const auto node_ids = requireSequence(buses[i], "expected_node_ids", context);
            if (node_ids.size() == 0)
            {
                throw std::invalid_argument("expected_node_ids must not be empty in " + context);
            }

            std::unordered_set<std::uint16_t> bus_node_ids;
            for (std::size_t node_index = 0; node_index < node_ids.size(); ++node_index)
            {
                const auto node_context = context + ".expected_node_ids[" + std::to_string(node_index) + "]";
                const auto node_id = parseNodeId(node_ids[node_index], node_context);
                if (!bus_node_ids.insert(node_id).second)
                {
                    throw std::invalid_argument("Duplicate node ID " + std::to_string(node_id) + " in " + context);
                }

                const auto owner = node_owners.find(node_id);
                if (owner != node_owners.end())
                {
                    throw std::invalid_argument("Node ID " + std::to_string(node_id) + " is declared by both logical buses '" +
                                                owner->second + "' and '" + bus.name + "'");
                }
                node_owners.emplace(node_id, bus.name);
                bus.expected_node_ids.push_back(node_id);
            }

            result.logical_buses.push_back(std::move(bus));
        }

        if (result.logical_buses.empty())
        {
            throw std::invalid_argument("can_buses must not be empty in " + config_path.string());
        }
        return result;
    }

    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path)
    {
        const auto root = YAML::LoadFile(config_path.string());
        const auto socketcan = requireMap(root, "socketcan", config_path.string());
        const auto discovery = requireMap(root, "discovery", config_path.string());

        DiscoveryOptions result;
        result.interface_regex = requireScalar<std::string>(socketcan, "interface_regex", config_path.string());
        result.require_interface_up = requireScalar<bool>(socketcan, "require_interface_up", config_path.string());

        const auto enabled = requireScalar<bool>(discovery, "enabled", config_path.string());
        if (!enabled)
        {
            throw std::invalid_argument("discovery.enabled must be true for ti5_can_discovery");
        }

        const auto strategy = requireScalar<std::string>(discovery, "strategy", config_path.string());
        if (strategy != "expected_node_ids")
        {
            throw std::invalid_argument("Unsupported discovery.strategy: " + strategy);
        }

        const auto response_timeout_ms = requireScalar<int>(discovery, "response_timeout_ms", config_path.string());
        result.response_timeout = std::chrono::milliseconds{response_timeout_ms};
        result.confirmations_required = requireScalar<std::size_t>(discovery, "confirmations_required", config_path.string());
        result.max_attempts = requireScalar<std::size_t>(discovery, "max_attempts", config_path.string());
        result.allow_partial_bus = requireScalar<bool>(discovery, "allow_partial_bus", config_path.string());
        result.require_unique_bus_match = requireScalar<bool>(discovery, "require_unique_bus_match", config_path.string());

        if (result.allow_partial_bus)
        {
            throw std::invalid_argument("discovery.allow_partial_bus must be false for ti5_can_discovery");
        }
        if (!result.require_unique_bus_match)
        {
            throw std::invalid_argument("discovery.require_unique_bus_match must be true for ti5_can_discovery");
        }
        validateDiscoverySettings(result, config_path.string());
        return result;
    }
}
