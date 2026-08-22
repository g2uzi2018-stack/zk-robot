#pragma once

#include "ti5/config/config.hpp"

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

    // 在调用方提供的本体候选接口中，通过 0x08 位置查询确认逻辑分区。
    // 不负责枚举 USB 设备、配置 bitrate、拉起接口或选择本体适配器。
    class CanDiscovery final
    {
    public:
        DiscoveryResult discover(const std::vector<LogicalCanBus> &logical_buses,
                                 const DiscoveryOptions &options,
                                 const std::vector<std::string> &candidate_interfaces) const;
    };
}
