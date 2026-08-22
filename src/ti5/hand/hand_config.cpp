#include "ti5/hand/hand_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

constexpr std::uint64_t kMaxUnsigned = 0xFFFFFFFFULL;

[[noreturn]] void throwConfigError(
    const std::string &context,
    const std::string &message)
{
    throw std::invalid_argument(context + ": " + message);
}

YAML::Node loadYamlFile(const std::filesystem::path &path)
{
    try
    {
        return YAML::LoadFile(path.string());
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            path.string(),
            "无法读取或解析 YAML 文件: " + std::string(error.what()));
    }
}

YAML::Node requireMap(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsMap())
    {
        throwConfigError(
            context + "." + key,
            "必须是 YAML mapping");
    }
    return value;
}

YAML::Node requireScalar(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsScalar())
    {
        throwConfigError(
            context + "." + key,
            "必须是 YAML scalar");
    }
    return value;
}

std::string requireString(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context)
{
    try
    {
        return requireScalar(parent, key, context).as<std::string>();
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "类型错误，期望字符串: " + std::string(error.what()));
    }
}

bool requireBool(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context)
{
    try
    {
        return requireScalar(parent, key, context).as<bool>();
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "类型错误，期望布尔值: " + std::string(error.what()));
    }
}

std::uint64_t requireUnsigned(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context)
{
    try
    {
        const auto value =
            requireScalar(parent, key, context).as<std::int64_t>();
        if (value < 0)
        {
            throwConfigError(context + "." + key, "不能为负数");
        }
        return static_cast<std::uint64_t>(value);
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "类型错误，期望非负整数: " + std::string(error.what()));
    }
}

std::uint64_t optionalUnsigned(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context,
    const std::uint64_t default_value)
{
    if (!parent[key])
    {
        return default_value;
    }
    return requireUnsigned(parent, key, context);
}

bool optionalBool(
    const YAML::Node &parent,
    const std::string &key,
    const std::string &context,
    const bool default_value)
{
    if (!parent[key])
    {
        return default_value;
    }
    return requireBool(parent, key, context);
}

std::size_t toSize(
    const std::uint64_t value,
    const std::string &context)
{
    if (value > static_cast<std::uint64_t>(
                    std::numeric_limits<std::size_t>::max()))
    {
        throwConfigError(context, "数值超出 size_t 范围");
    }
    return static_cast<std::size_t>(value);
}

std::chrono::milliseconds toMilliseconds(
    const std::uint64_t value,
    const std::string &context)
{
    using Rep = std::chrono::milliseconds::rep;
    if (value > static_cast<std::uint64_t>(
                    std::numeric_limits<Rep>::max()))
    {
        throwConfigError(context, "数值超出 milliseconds 范围");
    }
    return std::chrono::milliseconds{static_cast<Rep>(value)};
}

void validateAdapterSelector(
    const robot::can::CanAdapterSelector &selector,
    const std::string &context)
{
    if (selector.kind != "usb_serial_short" &&
        selector.kind != "usb_serial" &&
        selector.kind != "id_path" &&
        selector.kind != "sysfs_parent" &&
        selector.kind != "device_path")
    {
        throwConfigError(
            context + ".selector",
            "不支持的 CAN 适配器 selector");
    }
    if (selector.value.empty())
    {
        throwConfigError(context + ".value", "不能为空");
    }
    if (selector.expected_channels == 0)
    {
        throwConfigError(
            context + ".expected_channels",
            "必须为正数");
    }
}

robot::ti5::hand::HandSideConfig parseSide(
    const YAML::Node &hands,
    const std::string &side,
    const std::string &context)
{
    const auto side_node = requireMap(hands, side, context);
    const auto side_context = context + "." + side;

    robot::ti5::hand::HandSideConfig result;
    result.name = requireString(side_node, "name", side_context);
    result.protocol = requireString(side_node, "protocol", side_context);
    const auto node_id = requireUnsigned(
        side_node,
        "controller_node_id",
        side_context);
    if (node_id == 0 || node_id > 0xFF)
    {
        throwConfigError(
            side_context + ".controller_node_id",
            "傲意 HAND_ID 必须在 1..255 范围内");
    }
    result.controller_node_id = static_cast<std::uint8_t>(node_id);
    result.required_for_body_startup = requireBool(
        side_node,
        "required_for_body_startup",
        side_context);
    result.protocol_verified = requireBool(
        side_node,
        "protocol_verified",
        side_context);

    if (side_node["interface_selector"])
    {
        throwConfigError(
            side_context + ".interface_selector",
            "禁止使用 adapter_channel；接口必须由协议 Discovery 决定");
    }

    const auto discovery = requireMap(side_node, "discovery", side_context);
    result.discovery_enabled = requireBool(
        discovery,
        "enabled",
        side_context + ".discovery");

    const auto control = requireMap(side_node, "control", side_context);
    result.control_enabled = requireBool(
        control,
        "enabled",
        side_context + ".control");
    if (result.name.empty() || result.protocol != "aoyi_hand")
    {
        throwConfigError(
            side_context,
            "name 不能为空且 protocol 必须为 aoyi_hand");
    }
    return result;
}

} // namespace

namespace robot::ti5::hand
{

HandConfig loadHandConfig(const std::filesystem::path &config_path)
{
    const auto root = loadYamlFile(config_path);
    const auto context = config_path.string();
    if (!root || !root.IsMap())
    {
        throwConfigError(context, "根节点必须是 YAML mapping");
    }

    const auto schema = requireUnsigned(root, "schema_version", context);
    if (schema != 1)
    {
        throwConfigError(
            context + ".schema_version",
            "只支持 schema_version=1");
    }

    const auto transport = requireMap(root, "transport", context);
    const auto transport_context = context + ".transport";
    HandConfig result;
    result.transport.type = requireString(
        transport,
        "type",
        transport_context);
    if (result.transport.type != "socketcan")
    {
        throwConfigError(
            transport_context + ".type",
            "必须为 socketcan");
    }

    const auto bitrate = requireUnsigned(
        transport,
        "bitrate",
        transport_context);
    const auto restart_ms = requireUnsigned(
        transport,
        "restart_ms",
        transport_context);
    if (bitrate == 0 || bitrate > kMaxUnsigned)
    {
        throwConfigError(
            transport_context + ".bitrate",
            "必须在 1..4294967295 范围内");
    }
    if (restart_ms > kMaxUnsigned)
    {
        throwConfigError(
            transport_context + ".restart_ms",
            "超出 uint32_t 范围");
    }
    result.transport.bitrate = static_cast<std::uint32_t>(bitrate);
    result.transport.restart_ms = static_cast<std::uint32_t>(restart_ms);
    result.transport.manage_linux_link = requireBool(
        transport,
        "manage_linux_link",
        transport_context);
    result.transport.require_interface_up = optionalBool(
        transport,
        "require_interface_up",
        transport_context,
        true);
    result.transport.validate_bitrate = optionalBool(
        transport,
        "validate_bitrate",
        transport_context,
        true);
    result.transport.reconfigure_wait = toMilliseconds(
        optionalUnsigned(
            transport,
            "reconfigure_wait_ms",
            transport_context,
            100),
        transport_context + ".reconfigure_wait_ms");
    result.transport.startup_wait = toMilliseconds(
        optionalUnsigned(
            transport,
            "startup_wait_ms",
            transport_context,
            100),
        transport_context + ".startup_wait_ms");

    const auto adapter = requireMap(
        transport,
        "adapter_selector",
        transport_context);
    const auto adapter_context = transport_context + ".adapter_selector";
    result.transport.adapter_selector.kind = requireString(
        adapter,
        "selector",
        adapter_context);
    result.transport.adapter_selector.value = requireString(
        adapter,
        "value",
        adapter_context);
    result.transport.adapter_selector.expected_channels = toSize(
        requireUnsigned(adapter, "expected_channels", adapter_context),
        adapter_context + ".expected_channels");
    validateAdapterSelector(
        result.transport.adapter_selector,
        adapter_context);

    if (root["discovery"])
    {
        const auto discovery = requireMap(root, "discovery", context);
        const auto discovery_context = context + ".discovery";
        result.discovery.response_timeout = toMilliseconds(
            requireUnsigned(
                discovery,
                "response_timeout_ms",
                discovery_context),
            discovery_context + ".response_timeout_ms");
        result.discovery.confirmations_required = toSize(
            requireUnsigned(
                discovery,
                "confirmations_required",
                discovery_context),
            discovery_context + ".confirmations_required");
        result.discovery.max_attempts = toSize(
            requireUnsigned(
                discovery,
                "max_attempts",
                discovery_context),
            discovery_context + ".max_attempts");
    }
    if (result.discovery.response_timeout.count() <= 0 ||
        result.discovery.confirmations_required == 0 ||
        result.discovery.max_attempts == 0 ||
        result.discovery.max_attempts <
            result.discovery.confirmations_required)
    {
        throwConfigError(
            context + ".discovery",
            "Discovery timeout/retry 配置无效");
    }

    const auto hands = requireMap(root, "hands", context);
    result.left = parseSide(hands, "left", context + ".hands");
    result.right = parseSide(hands, "right", context + ".hands");
    if (result.left.controller_node_id == result.right.controller_node_id)
    {
        throwConfigError(
            context + ".hands",
            "左右手 controller_node_id 必须不同");
    }
    return result;
}

} // namespace robot::ti5::hand
