#pragma once

#include "ti5/config/config.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::ti5
{
    // Discovery 原型沿用独立配置层的逻辑总线对象，不反向引入运行时依赖。
    using LogicalCanBus = LogicalCanBusConfig;

    // Discovery 使用的运行时参数。该结构不依赖 YAML，便于协议和扫描逻辑独立演进。
    struct DiscoveryOptions
    {
        std::string interface_regex;
        bool require_interface_up{true};
        std::chrono::milliseconds response_timeout{50};
        std::size_t confirmations_required{3};
        std::size_t max_attempts{5};
        bool allow_partial_bus{false};
        bool require_unique_bus_match{true};
    };

    struct InterfaceDiscoveryResult
    {
        std::string interface_name;
        std::vector<std::uint16_t> confirmed_node_ids;
        bool opened{false};
        std::string error;
    };

    struct LogicalBusDiscoveryResult
    {
        std::string bus_name;
        bool complete{false};
        std::optional<std::string> interface_name;
        std::vector<std::uint16_t> matched_node_ids;
        std::vector<std::uint16_t> missing_node_ids;
    };

    struct DiscoveryResult
    {
        bool success{false};
        std::vector<InterfaceDiscoveryResult> interfaces;
        std::vector<LogicalBusDiscoveryResult> logical_buses;
    };

    // 枚举 UP 的 canX 接口，并通过 0x08 位置查询确认本体逻辑分区。
    // 该类只依赖中性的 DiscoveryOptions/LogicalCanBus，不直接依赖 YAML。
    class CanDiscovery final
    {
    public:
        DiscoveryResult discover(const std::vector<LogicalCanBus> &logical_buses,
                                 const DiscoveryOptions &options) const;
    };
}
