#include "ti5/config/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace
{
    constexpr std::uint64_t kSupportedSchemaVersion = 1;
    constexpr std::uint64_t kMaxCanNodeId = 0x7FF;

    [[noreturn]] void throwConfigError(const std::string &context, const std::string &message)
    {
        throw std::invalid_argument(context + ": " + message);
    }

    YAML::Node loadYamlFile(const std::filesystem::path &config_path)
    {
        try
        {
            return YAML::LoadFile(config_path.string());
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(config_path.string(), "无法读取或解析 YAML 文件: " + std::string(error.what()));
        }
    }

    YAML::Node requireField(const YAML::Node &map,
                            const std::string &key,
                            const std::string &context)
    {
        if (!map || !map.IsMap())
        {
            throwConfigError(context, "必须是 YAML mapping");
        }

        const auto value = map[key];
        if (!value)
        {
            throwConfigError(context, "缺少必需字段 '" + key + "'");
        }
        if (value.IsNull())
        {
            throwConfigError(context + "." + key, "不能是 null");
        }
        return value;
    }

    YAML::Node requireMap(const YAML::Node &map,
                          const std::string &key,
                          const std::string &context)
    {
        const auto value = requireField(map, key, context);
        if (!value.IsMap())
        {
            throwConfigError(context + "." + key, "类型错误，期望 YAML mapping");
        }
        return value;
    }

    YAML::Node requireSequence(const YAML::Node &map,
                               const std::string &key,
                               const std::string &context)
    {
        const auto value = requireField(map, key, context);
        if (!value.IsSequence())
        {
            throwConfigError(context + "." + key, "类型错误，期望 YAML sequence");
        }
        return value;
    }

    YAML::Node requireScalarNode(const YAML::Node &map,
                                 const std::string &key,
                                 const std::string &context)
    {
        const auto value = requireField(map, key, context);
        if (!value.IsScalar())
        {
            throwConfigError(context + "." + key, "类型错误，期望 YAML scalar");
        }
        return value;
    }

    bool hasStringTag(const YAML::Node &node)
    {
        const auto tag = node.Tag();
        return tag == "!" || tag == "tag:yaml.org,2002:str";
    }

    bool hasExplicitNonStringTag(const YAML::Node &node)
    {
        const auto tag = node.Tag();
        return !tag.empty() && tag != "?" && tag != "!" && tag != "tag:yaml.org,2002:str";
    }

    bool isImplicitTypedScalar(const YAML::Node &node)
    {
        if (node.Tag() != "?")
        {
            return false;
        }

        const auto scalar = node.Scalar();
        const auto lower = [&scalar]() {
            std::string value = scalar;
            for (auto &character : value)
            {
                if (character >= 'A' && character <= 'Z')
                {
                    character = static_cast<char>(character - 'A' + 'a');
                }
            }
            return value;
        }();

        if (lower == "true" || lower == "false" || lower == "yes" || lower == "no" || lower == "on" ||
            lower == "off" || lower == "null" || lower == "~")
        {
            return true;
        }

        try
        {
            static_cast<void>(node.as<std::int64_t>());
            return true;
        }
        catch (const YAML::Exception &)
        {
        }

        try
        {
            static_cast<void>(node.as<double>());
            return true;
        }
        catch (const YAML::Exception &)
        {
            return false;
        }
    }

    std::string requireString(const YAML::Node &map,
                              const std::string &key,
                              const std::string &context)
    {
        const auto value = requireScalarNode(map, key, context);
        if (hasExplicitNonStringTag(value) || isImplicitTypedScalar(value))
        {
            throwConfigError(context + "." + key, "类型错误，期望字符串");
        }

        try
        {
            return value.as<std::string>();
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(context + "." + key, "类型错误，期望字符串: " + std::string(error.what()));
        }
    }

    bool requireBool(const YAML::Node &map,
                     const std::string &key,
                     const std::string &context)
    {
        const auto value = requireScalarNode(map, key, context);
        if (hasStringTag(value))
        {
            throwConfigError(context + "." + key, "类型错误，期望布尔值");
        }

        try
        {
            return value.as<bool>();
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(context + "." + key, "类型错误，期望布尔值: " + std::string(error.what()));
        }
    }

    std::uint64_t requireUnsignedScalar(const YAML::Node &value,
                                        const std::string &context)
    {
        if (!value || value.IsNull() || !value.IsScalar())
        {
            throwConfigError(context, "类型错误，期望非负整数 scalar");
        }
        if (hasStringTag(value))
        {
            throwConfigError(context, "类型错误，期望非负整数");
        }

        try
        {
            const auto signed_value = value.as<std::int64_t>();
            if (signed_value < 0)
            {
                throwConfigError(context, "数值不能为负数");
            }
            return static_cast<std::uint64_t>(signed_value);
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(context, "类型错误，期望非负整数: " + std::string(error.what()));
        }
    }

    std::uint64_t requireUnsigned(const YAML::Node &map,
                                  const std::string &key,
                                  const std::string &context)
    {
        return requireUnsignedScalar(requireScalarNode(map, key, context), context + "." + key);
    }

    std::size_t toSizeT(const std::uint64_t value, const std::string &context)
    {
        if (value > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
        {
            throwConfigError(context, "数值超出 size_t 范围");
        }
        return static_cast<std::size_t>(value);
    }

    std::chrono::milliseconds toMilliseconds(const std::uint64_t value,
                                             const std::string &context)
    {
        using Rep = std::chrono::milliseconds::rep;
        if (value > static_cast<std::uint64_t>(std::numeric_limits<Rep>::max()))
        {
            throwConfigError(context, "数值超出 milliseconds 范围");
        }
        return std::chrono::milliseconds{static_cast<Rep>(value)};
    }

    void requireSchemaVersion(const YAML::Node &root,
                              const std::string &context)
    {
        const auto version = requireUnsigned(root, "schema_version", context);
        if (version != kSupportedSchemaVersion)
        {
            throwConfigError(context + ".schema_version",
                             "不支持的版本 " + std::to_string(version) + ", 只支持 " +
                                 std::to_string(kSupportedSchemaVersion));
        }
    }

    std::uint16_t parseNodeId(const YAML::Node &node, const std::string &context)
    {
        const auto value = requireUnsignedScalar(node, context);
        if (value == 0 || value > kMaxCanNodeId)
        {
            throwConfigError(context, "node ID 必须在 1..2047 范围内");
        }
        return static_cast<std::uint16_t>(value);
    }

    void validateRegex(const std::string &expression, const std::string &context)
    {
        try
        {
            static_cast<void>(std::regex{expression});
        }
        catch (const std::regex_error &error)
        {
            throwConfigError(context, "interface_regex 不是合法正则表达式: " + std::string(error.what()));
        }
    }

    void validateAdapterSelector(const robot::ti5::CanAdapterSelectorConfig &selector,
                                 const std::string &context)
    {
        if (selector.selector != "usb_serial_short" &&
            selector.selector != "usb_serial" &&
            selector.selector != "id_path" &&
            selector.selector != "sysfs_parent" &&
            selector.selector != "device_path")
        {
            throwConfigError(context + ".selector",
                             "只支持 usb_serial_short、usb_serial、id_path、sysfs_parent、device_path");
        }
        if (selector.value.empty())
        {
            throwConfigError(context + ".value", "不能为空");
        }
        if (selector.expected_channels == 0)
        {
            throwConfigError(context + ".expected_channels", "必须为正数");
        }
    }
}

namespace robot::ti5
{
    T170cRobotConfig loadRobotConfig(const std::filesystem::path &config_path)
    {
        const auto root = loadYamlFile(config_path);
        const auto context = config_path.string();
        if (!root || !root.IsMap())
        {
            throwConfigError(context, "根节点必须是 YAML mapping");
        }
        requireSchemaVersion(root, context);

        const auto robot = requireMap(root, "robot", context);
        const auto robot_context = context + ".robot";

        T170cRobotConfig result;
        result.vendor = requireString(robot, "vendor", robot_context);
        result.model = requireString(robot, "model", robot_context);
        result.body_motor_count = toSizeT(requireUnsigned(robot, "body_motor_count", robot_context),
                                          robot_context + ".body_motor_count");

        if (result.vendor.empty())
        {
            throwConfigError(robot_context + ".vendor", "不能为空");
        }
        if (result.model != "T170C")
        {
            throwConfigError(robot_context + ".model", "必须为 T170C");
        }
        if (result.body_motor_count == 0)
        {
            throwConfigError(robot_context + ".body_motor_count", "必须为正数");
        }

        // 现有 robot.yaml 将逻辑 CAN 分区放在根节点，与 robot 元数据同级。
        const auto buses = requireSequence(root, "can_buses", context);
        if (buses.size() == 0)
        {
            throwConfigError(context + ".can_buses", "不能为空");
        }

        std::unordered_set<std::string> bus_names;
        std::size_t total_node_count = 0;
        for (std::size_t bus_index = 0; bus_index < buses.size(); ++bus_index)
        {
            const auto bus_context = context + ".can_buses[" + std::to_string(bus_index) + "]";
            if (!buses[bus_index].IsMap())
            {
                throwConfigError(bus_context, "类型错误，期望 YAML mapping");
            }

            const auto &bus_node = buses[bus_index];
            LogicalCanBusConfig bus;
            bus.name = requireString(bus_node, "name", bus_context);
            bus.protocol = requireString(bus_node, "protocol", bus_context);
            bus.required = requireBool(bus_node, "required", bus_context);

            if (bus.name.empty())
            {
                throwConfigError(bus_context + ".name", "不能为空");
            }
            if (bus.protocol.empty())
            {
                throwConfigError(bus_context + ".protocol", "不能为空");
            }
            if (!bus_names.insert(bus.name).second)
            {
                throwConfigError(bus_context + ".name", "逻辑 bus 名称重复: '" + bus.name + "'");
            }

            const auto node_ids = requireSequence(bus_node, "expected_node_ids", bus_context);
            if (node_ids.size() == 0)
            {
                throwConfigError(bus_context + ".expected_node_ids", "不能为空");
            }
            if (node_ids.size() > std::numeric_limits<std::size_t>::max() - total_node_count)
            {
                throwConfigError(bus_context + ".expected_node_ids", "node ID 总数溢出");
            }

            // node ID 在逻辑 CAN 总线内必须唯一；不同 CAN 总线可以使用相同的 node ID。
            std::unordered_set<std::uint16_t> bus_node_ids;
            for (std::size_t node_index = 0; node_index < node_ids.size(); ++node_index)
            {
                const auto node_context = bus_context + ".expected_node_ids[" + std::to_string(node_index) + "]";
                const auto node_id = parseNodeId(node_ids[node_index], node_context);
                if (!bus_node_ids.insert(node_id).second)
                {
                    throwConfigError(node_context, "同一逻辑 bus 内 node ID 重复: " + std::to_string(node_id));
                }
                bus.expected_node_ids.push_back(node_id);
            }

            total_node_count += bus.expected_node_ids.size();
            result.can_buses.push_back(std::move(bus));
        }

        if (total_node_count != result.body_motor_count)
        {
            throwConfigError(robot_context + ".body_motor_count",
                             "声明值 " + std::to_string(result.body_motor_count) +
                                 " 与所有逻辑 bus 的 node ID 总数 " + std::to_string(total_node_count) + " 不一致");
        }
        return result;
    }

    CanConfig loadCanConfig(const std::filesystem::path &config_path)
    {
        const auto root = loadYamlFile(config_path);
        const auto context = config_path.string();
        if (!root || !root.IsMap())
        {
            throwConfigError(context, "根节点必须是 YAML mapping");
        }
        requireSchemaVersion(root, context);

        CanConfig result;
        const auto socketcan = requireMap(root, "socketcan", context);
        const auto socketcan_context = context + ".socketcan";
        const auto bitrate = requireUnsigned(socketcan, "bitrate", socketcan_context);
        if (bitrate == 0 || bitrate > std::numeric_limits<std::uint32_t>::max())
        {
            throwConfigError(socketcan_context + ".bitrate", "必须在 1..4294967295 范围内");
        }
        result.socketcan.bitrate = static_cast<std::uint32_t>(bitrate);
        result.socketcan.interface_regex = requireString(socketcan, "interface_regex", socketcan_context);
        result.socketcan.require_interface_up = requireBool(socketcan, "require_interface_up", socketcan_context);
        result.socketcan.validate_bitrate = requireBool(socketcan, "validate_bitrate", socketcan_context);
        result.socketcan.manage_linux_link = requireBool(socketcan, "manage_linux_link", socketcan_context);
        const auto restart_ms = requireUnsigned(socketcan, "restart_ms", socketcan_context);
        result.socketcan.restart_ms = toMilliseconds(restart_ms, socketcan_context + ".restart_ms");
        const auto reconfigure_wait_ms = requireUnsigned(socketcan, "reconfigure_wait_ms", socketcan_context);
        result.socketcan.reconfigure_wait =
            toMilliseconds(reconfigure_wait_ms, socketcan_context + ".reconfigure_wait_ms");
        const auto startup_wait_ms = requireUnsigned(socketcan, "startup_wait_ms", socketcan_context);
        result.socketcan.startup_wait =
            toMilliseconds(startup_wait_ms, socketcan_context + ".startup_wait_ms");

        const auto body_adapter = requireMap(socketcan, "body_adapter", socketcan_context);
        const auto body_adapter_context = socketcan_context + ".body_adapter";
        result.socketcan.body_adapter.selector =
            requireString(body_adapter, "selector", body_adapter_context);
        result.socketcan.body_adapter.value =
            requireString(body_adapter, "value", body_adapter_context);
        result.socketcan.body_adapter.expected_channels =
            toSizeT(requireUnsigned(body_adapter, "expected_channels", body_adapter_context),
                    body_adapter_context + ".expected_channels");
        validateAdapterSelector(result.socketcan.body_adapter, body_adapter_context);

        if (result.socketcan.interface_regex.empty())
        {
            throwConfigError(socketcan_context + ".interface_regex", "不能为空");
        }
        validateRegex(result.socketcan.interface_regex, socketcan_context);

        const auto discovery = requireMap(root, "discovery", context);
        const auto discovery_context = context + ".discovery";
        result.discovery.enabled = requireBool(discovery, "enabled", discovery_context);

        const auto response_timeout_ms = requireUnsigned(discovery, "response_timeout_ms", discovery_context);
        if (response_timeout_ms == 0)
        {
            throwConfigError(discovery_context + ".response_timeout_ms", "必须为正数");
        }
        result.discovery.response_timeout =
            toMilliseconds(response_timeout_ms, discovery_context + ".response_timeout_ms");

        result.discovery.confirmations_required =
            toSizeT(requireUnsigned(discovery, "confirmations_required", discovery_context),
                    discovery_context + ".confirmations_required");
        result.discovery.max_attempts =
            toSizeT(requireUnsigned(discovery, "max_attempts", discovery_context),
                    discovery_context + ".max_attempts");
        result.discovery.allow_partial_bus = requireBool(discovery, "allow_partial_bus", discovery_context);
        result.discovery.require_unique_bus_match =
            requireBool(discovery, "require_unique_bus_match", discovery_context);

        if (result.discovery.confirmations_required == 0)
        {
            throwConfigError(discovery_context + ".confirmations_required", "必须为正数");
        }
        if (result.discovery.max_attempts == 0)
        {
            throwConfigError(discovery_context + ".max_attempts", "必须为正数");
        }
        if (result.discovery.max_attempts < result.discovery.confirmations_required)
        {
            throwConfigError(discovery_context + ".max_attempts",
                             "必须不小于 confirmations_required (" +
                                 std::to_string(result.discovery.confirmations_required) + ")");
        }
        return result;
    }

    DiscoveryOptions makeDiscoveryOptions(const CanConfig &config)
    {
        const auto &socketcan = config.socketcan;
        const auto &discovery = config.discovery;

        if (socketcan.interface_regex != "^can[0-9]+$")
        {
            throw std::invalid_argument(
                "socketcan.interface_regex must be '^can[0-9]+$'");
        }
        if (!socketcan.require_interface_up)
        {
            throw std::invalid_argument(
                "socketcan.require_interface_up must be true for T170C discovery");
        }
        if (!discovery.enabled)
        {
            throw std::invalid_argument(
                "discovery.enabled must be true for ti5_can_discovery");
        }
        if (discovery.allow_partial_bus)
        {
            throw std::invalid_argument(
                "discovery.allow_partial_bus must be false for ti5_can_discovery");
        }
        if (!discovery.require_unique_bus_match)
        {
            throw std::invalid_argument(
                "discovery.require_unique_bus_match must be true for ti5_can_discovery");
        }

        DiscoveryOptions result;
        result.interface_regex = socketcan.interface_regex;
        result.require_interface_up = socketcan.require_interface_up;
        result.response_timeout = discovery.response_timeout;
        result.confirmations_required = discovery.confirmations_required;
        result.max_attempts = discovery.max_attempts;
        result.allow_partial_bus = discovery.allow_partial_bus;
        result.require_unique_bus_match = discovery.require_unique_bus_match;
        return result;
    }

    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path)
    {
        return makeDiscoveryOptions(loadCanConfig(config_path));
    }
}
