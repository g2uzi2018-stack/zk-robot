#pragma once

#include "ti5/config.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::ti5
{
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
