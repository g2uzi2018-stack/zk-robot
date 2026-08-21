#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace robot::ti5
{
    // robot.yaml 中的一条逻辑 CAN 分区。node ID 的作用域是本逻辑总线。
    struct LogicalCanBus
    {
        std::string name;
        std::string protocol;
        bool required{false};
        std::vector<std::uint16_t> expected_node_ids;
    };

    // can.yaml 中 Discovery 使用的运行时参数。该结构不执行 Discovery。
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

    // T170C 本体配置。其余 joints、encoder_defaults 等段留给后续专用配置层。
    struct Ti5RobotConfig
    {
        std::string vendor;
        std::string model;
        std::size_t body_motor_count{0};
        std::vector<LogicalCanBus> can_buses;
    };

    // 保持配置层现有公开类型名兼容；新代码使用中性的共享类型。
    using LogicalCanBusConfig = LogicalCanBus;
    using T170cRobotConfig = Ti5RobotConfig;

    // can.yaml 中当前配置加载阶段需要的 Discovery 参数。
    struct CanDiscoveryConfig
    {
        bool enabled{false};
        std::chrono::milliseconds response_timeout{0};
        std::size_t confirmations_required{0};
        std::size_t max_attempts{0};
        bool allow_partial_bus{false};
        bool require_unique_bus_match{false};
    };

    // can.yaml 中当前配置加载阶段需要的 SocketCAN 参数。
    struct SocketCanConfig
    {
        std::uint32_t bitrate{0};
        std::string interface_regex;
        bool require_interface_up{false};
        bool validate_bitrate{false};
        bool manage_linux_link{false};
        std::chrono::milliseconds restart_ms{0};
    };

    // CAN 配置对象只包含当前阶段所需的 SocketCAN 和 Discovery 参数。
    struct CanConfig
    {
        SocketCanConfig socketcan;
        CanDiscoveryConfig discovery;
    };
}
