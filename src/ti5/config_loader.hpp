#pragma once

#include "ti5/discovery.hpp"

#include <filesystem>

namespace robot::ti5
{
    struct Ti5RobotConfig
    {
        std::vector<LogicalCanBus> logical_buses;
    };

    Ti5RobotConfig loadRobotConfig(const std::filesystem::path &config_path);

    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path);
}
