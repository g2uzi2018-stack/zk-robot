#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot::ti5
{

struct LogicalCanBus
{
    std::string name;
    std::string protocol;
    bool required{false};
    std::vector<std::uint16_t> expected_node_ids;
};

struct DiscoveryOptions
{
    // 保留为运行时配置校验字段；接口枚举由 CanInterfaceManager 完成。
    std::string interface_regex;
    bool require_interface_up{true};

    std::chrono::milliseconds response_timeout{50};
    std::size_t confirmations_required{3};
    std::size_t max_attempts{5};

    bool allow_partial_bus{false};
    bool require_unique_bus_match{true};
};

struct Ti5RobotConfig
{
    std::string vendor;
    std::string model;
    std::size_t body_motor_count{0};
    std::vector<LogicalCanBus> can_buses;
};

using LogicalCanBusConfig = LogicalCanBus;
using T170cRobotConfig = Ti5RobotConfig;

struct CanDiscoveryConfig
{
    bool enabled{false};
    std::chrono::milliseconds response_timeout{0};
    std::size_t confirmations_required{0};
    std::size_t max_attempts{0};
    bool allow_partial_bus{false};
    bool require_unique_bus_match{false};
};

// 只描述“哪一块物理 USB-CAN 适配器属于本体”，不描述 waist/head/arm 对应哪个 canX。
struct CanAdapterSelectorConfig
{
    // 支持：usb_serial_short、usb_serial、id_path、sysfs_parent、device_path。
    std::string selector;
    std::string value;
    std::size_t expected_channels{0};
};

struct SocketCanConfig
{
    std::uint32_t bitrate{0};
    std::string interface_regex;
    bool require_interface_up{false};
    bool validate_bitrate{false};
    bool manage_linux_link{false};
    std::chrono::milliseconds restart_ms{0};
    std::chrono::milliseconds reconfigure_wait{100};
    std::chrono::milliseconds startup_wait{100};
    CanAdapterSelectorConfig body_adapter;
};

struct CanConfig
{
    SocketCanConfig socketcan;
    CanDiscoveryConfig discovery;
};

} // namespace robot::ti5
