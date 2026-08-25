#include "ti5/config/config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
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

    double requireDouble(const YAML::Node &map,
                         const std::string &key,
                         const std::string &context)
    {
        const auto value = requireScalarNode(map, key, context);
        if (hasStringTag(value) || hasExplicitNonStringTag(value))
        {
            throwConfigError(context + "." + key, "类型错误，期望浮点数");
        }
        try
        {
            const auto parsed = value.as<double>();
            if (!std::isfinite(parsed))
            {
                throwConfigError(context + "." + key, "数值必须为有限值");
            }
            return parsed;
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(context + "." + key,
                             "类型错误，期望浮点数: " + std::string(error.what()));
        }
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

    std::chrono::microseconds toMicroseconds(const std::uint64_t value,
                                             const std::string &context)
    {
        using Rep = std::chrono::microseconds::rep;
        if (value > static_cast<std::uint64_t>(std::numeric_limits<Rep>::max()))
        {
            throwConfigError(context, "数值超出 microseconds 范围");
        }
        return std::chrono::microseconds{static_cast<Rep>(value)};
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
        if (result.body_motor_count != 22)
        {
            throwConfigError(robot_context + ".body_motor_count", "T170C 本体必须为 22 个 motor");
        }

        const auto encoder_defaults = requireMap(root, "encoder_defaults", context);
        const auto encoder_context = context + ".encoder_defaults";
        result.encoder_defaults.type = requireString(encoder_defaults, "type", encoder_context);
        result.encoder_defaults.position_reference =
            requireString(encoder_defaults, "position_reference", encoder_context);
        const auto counts_per_output_revolution = requireUnsigned(
            encoder_defaults,
            "counts_per_output_revolution",
            encoder_context);
        if (counts_per_output_revolution > std::numeric_limits<std::uint32_t>::max())
        {
            throwConfigError(encoder_context + ".counts_per_output_revolution",
                             "数值超出 uint32_t 范围");
        }
        result.encoder_defaults.counts_per_output_revolution =
            static_cast<std::uint32_t>(counts_per_output_revolution);
        result.encoder_defaults.gear_ratio =
            requireDouble(encoder_defaults, "gear_ratio", encoder_context);

        if (result.encoder_defaults.type != "dual")
        {
            throwConfigError(encoder_context + ".type", "当前 T170C 只支持 dual 编码器");
        }
        if (result.encoder_defaults.position_reference != "output")
        {
            throwConfigError(encoder_context + ".position_reference", "当前 T170C 位置参考必须为 output");
        }
        if (result.encoder_defaults.counts_per_output_revolution == 0)
        {
            throwConfigError(encoder_context + ".counts_per_output_revolution", "必须大于 0");
        }
        if (!(result.encoder_defaults.gear_ratio > 0.0) ||
            !std::isfinite(result.encoder_defaults.gear_ratio))
        {
            throwConfigError(encoder_context + ".gear_ratio", "必须为正数");
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

        const auto joints = requireSequence(root, "joints", context);
        if (joints.size() != result.body_motor_count)
        {
            throwConfigError(context + ".joints", "physical joint 数量必须等于 body_motor_count=22");
        }

        std::unordered_set<std::string> joint_names;
        std::unordered_set<std::string> physical_names;
        std::unordered_set<std::string> physical_joint_keys;
        std::size_t node_two_count = 0;

        for (std::size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
        {
            const auto joint_context = context + ".joints[" +
                                       std::to_string(joint_index) + "]";
            if (!joints[joint_index].IsMap())
            {
                throwConfigError(joint_context, "类型错误，期望 YAML mapping");
            }

            const auto &joint_node = joints[joint_index];
            PhysicalJointConfig joint;
            joint.name = requireString(joint_node, "name", joint_context);
            joint.physical_name = requireString(joint_node, "physical_name", joint_context);
            joint.bus = requireString(joint_node, "bus", joint_context);

            if (joint.name.empty() || joint.physical_name.empty() || joint.bus.empty())
            {
                throwConfigError(joint_context, "name、physical_name、bus 不能为空");
            }
            if (!joint_names.insert(joint.name).second)
            {
                throwConfigError(joint_context + ".name", "physical joint 名称重复: " + joint.name);
            }
            if (!physical_names.insert(joint.physical_name).second)
            {
                throwConfigError(joint_context + ".physical_name",
                                 "physical joint 名称重复: " + joint.physical_name);
            }

            const auto bus = std::find_if(
                result.can_buses.begin(),
                result.can_buses.end(),
                [&joint](const auto &candidate) { return candidate.name == joint.bus; });
            if (bus == result.can_buses.end())
            {
                throwConfigError(joint_context + ".bus",
                                 "引用了未声明的 logical bus: " + joint.bus);
            }

            const auto motor = requireMap(joint_node, "motor", joint_context);
            const auto motor_context = joint_context + ".motor";
            joint.motor.node_id = parseNodeId(
                requireScalarNode(motor, "node_id", motor_context),
                motor_context + ".node_id");
            const auto unit = requireString(motor, "unit", motor_context);
            if (unit != "radian")
            {
                throwConfigError(motor_context + ".unit", "当前 T170C 只支持 radian");
            }
            joint.motor.unit = JointUnit::Radian;
            joint.motor.encoder = result.encoder_defaults;

            if (std::find(bus->expected_node_ids.begin(),
                          bus->expected_node_ids.end(),
                          joint.motor.node_id) == bus->expected_node_ids.end())
            {
                throwConfigError(motor_context + ".node_id",
                                 "node ID 不属于对应 logical bus 的 expected_node_ids");
            }

            if (joint.motor.node_id == 2 && ++node_two_count > 1)
            {
                throwConfigError(motor_context + ".node_id",
                                 "ID 2 只能存在一个 physical joint");
            }
            const auto physical_key = joint.bus + "#" +
                                      std::to_string(joint.motor.node_id);
            if (!physical_joint_keys.insert(physical_key).second)
            {
                throwConfigError(motor_context + ".node_id",
                                 "同一 logical bus 内 node ID 只能对应一个 physical joint");
            }
            result.joints.push_back(std::move(joint));
        }

        for (const auto &bus : result.can_buses)
        {
            for (const auto node_id : bus.expected_node_ids)
            {
                const auto physical_key = bus.name + "#" + std::to_string(node_id);
                if (physical_joint_keys.find(physical_key) == physical_joint_keys.end())
                {
                    throwConfigError(context + ".joints",
                                     "expected node 必须恰好对应一个 physical joint: " +
                                         bus.name + "/" + std::to_string(node_id));
                }
            }
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
        result.discovery.cache_mapping =
            requireBool(discovery, "cache_mapping", discovery_context);
        result.discovery.strategy =
            requireString(discovery, "strategy", discovery_context);

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
        result.discovery.discover_hands =
            requireBool(discovery, "discover_hands", discovery_context);

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

        if (result.discovery.cache_mapping)
        {
            throwConfigError(discovery_context + ".cache_mapping",
                             "当前 TI5 启动必须每次重新发现接口");
        }
        if (result.discovery.strategy != "expected_node_ids")
        {
            throwConfigError(discovery_context + ".strategy",
                             "当前只支持 expected_node_ids");
        }
        if (result.discovery.discover_hands)
        {
            throwConfigError(discovery_context + ".discover_hands",
                             "傲意手不能参与本体节点发现");
        }

        const auto receive = requireMap(root, "receive", context);
        const auto receive_context = context + ".receive";
        result.receive.centralized_receiver =
            requireBool(receive, "centralized_receiver", receive_context);
        result.receive.latest_feedback_cache =
            requireBool(receive, "latest_feedback_cache", receive_context);
        result.receive.timestamp_clock =
            requireString(receive, "timestamp_clock", receive_context);
        result.receive.use_can_filters =
            requireBool(receive, "use_can_filters", receive_context);
        result.receive.receive_error_frames =
            requireBool(receive, "receive_error_frames", receive_context);
        if (!result.receive.centralized_receiver ||
            !result.receive.latest_feedback_cache ||
            !result.receive.use_can_filters)
        {
            throwConfigError(receive_context,
                             "TI5 正式总线必须统一接收、缓存最新反馈并使用节点过滤");
        }
        if (result.receive.timestamp_clock != "monotonic")
        {
            throwConfigError(receive_context + ".timestamp_clock",
                             "必须为 monotonic");
        }

        const auto control = requireMap(root, "control", context);
        const auto control_context = context + ".control";
        result.control.frequency_hz = toSizeT(
            requireUnsigned(control, "frequency_hz", control_context),
            control_context + ".frequency_hz");
        result.control.inter_frame_gap = toMicroseconds(
            requireUnsigned(control, "inter_frame_gap_us", control_context),
            control_context + ".inter_frame_gap_us");
        result.control.post_batch_feedback_wait = toMicroseconds(
            requireUnsigned(control,
                            "post_batch_feedback_wait_us",
                            control_context),
            control_context + ".post_batch_feedback_wait_us");
        result.control.send_failure_threshold = toSizeT(
            requireUnsigned(control,
                            "send_failure_threshold",
                            control_context),
            control_context + ".send_failure_threshold");
        if (result.control.frequency_hz == 0 ||
            result.control.send_failure_threshold == 0)
        {
            throwConfigError(control_context,
                             "frequency_hz 和 send_failure_threshold 必须为正数");
        }

        const auto watchdog = requireMap(root, "watchdog", context);
        const auto watchdog_context = context + ".watchdog";
        result.watchdog.stale_feedback_cycles = toSizeT(
            requireUnsigned(watchdog,
                            "stale_feedback_cycles",
                            watchdog_context),
            watchdog_context + ".stale_feedback_cycles");
        result.watchdog.reject_new_motion_on_stale_feedback =
            requireBool(watchdog,
                        "reject_new_motion_on_stale_feedback",
                        watchdog_context);
        result.watchdog.enter_fault_on_stale_feedback =
            requireBool(watchdog,
                        "enter_fault_on_stale_feedback",
                        watchdog_context);
        result.watchdog.enter_fault_on_bus_off =
            requireBool(watchdog,
                        "enter_fault_on_bus_off",
                        watchdog_context);
        if (result.watchdog.stale_feedback_cycles == 0)
        {
            throwConfigError(watchdog_context + ".stale_feedback_cycles",
                             "必须为正数");
        }
        if (result.watchdog.enter_fault_on_bus_off &&
            !result.receive.receive_error_frames)
        {
            throwConfigError(watchdog_context + ".enter_fault_on_bus_off",
                             "启用总线关闭保护时必须接收 CAN 错误帧");
        }

        const auto exclusive_control =
            requireMap(root, "exclusive_control", context);
        const auto exclusive_context = context + ".exclusive_control";
        result.exclusive_control.enabled =
            requireBool(exclusive_control, "enabled", exclusive_context);
        result.exclusive_control.lock_file =
            requireString(exclusive_control, "lock_file", exclusive_context);
        result.exclusive_control.reject_second_controller =
            requireBool(exclusive_control,
                        "reject_second_controller",
                        exclusive_context);
        if (result.exclusive_control.enabled &&
            (result.exclusive_control.lock_file.empty() ||
             !result.exclusive_control.reject_second_controller))
        {
            throwConfigError(exclusive_context,
                             "启用独占控制时必须提供锁文件并拒绝第二个控制进程");
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
