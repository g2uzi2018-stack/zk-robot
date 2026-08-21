#include "common/logger.hpp"
#include "ti5/config_loader.hpp"
#include "ti5/discovery.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <exception>
#include <sstream>
#include <string>
#include <vector>

namespace
{
    std::string formatNodeIds(const std::vector<std::uint16_t> &node_ids)
    {
        if (node_ids.empty())
        {
            return "<none>";
        }

        std::ostringstream output;
        for (std::size_t i = 0; i < node_ids.size(); ++i)
        {
            if (i != 0)
            {
                output << ',';
            }
            output << node_ids[i];
        }
        return output.str();
    }

    void printDiscoverySummary(const robot::ti5::DiscoveryResult &result)
    {
        for (const auto &interface_result : result.interfaces)
        {
            if (!interface_result.error.empty())
            {
                robot::common::logger()->error("Scan result {}: ERROR {}",
                                               interface_result.interface_name,
                                               interface_result.error);
            }
            else
            {
                robot::common::logger()->info("Scan result {}: confirmed node IDs [{}]",
                                             interface_result.interface_name,
                                             formatNodeIds(interface_result.confirmed_node_ids));
            }
        }

        for (const auto &bus_result : result.logical_buses)
        {
            if (bus_result.complete && bus_result.interface_name)
            {
                robot::common::logger()->info("Logical bus mapping: {} -> {}",
                                             bus_result.bus_name,
                                             *bus_result.interface_name);
            }
            else
            {
                robot::common::logger()->error("Logical bus {} has no complete mapping; missing node IDs [{}]",
                                               bus_result.bus_name,
                                               formatNodeIds(bus_result.missing_node_ids));
            }
        }
    }
}

int main(int argc, char **argv)
{
    try
    {
        if (argc != 1 && argc != 3)
        {
            robot::common::logger()->error("Usage: {} [robot.yaml can.yaml]", argv[0]);
            return 2;
        }

        const std::filesystem::path robot_config_path = argc == 3 ? argv[1] : "config/ti5/t170c/robot.yaml";
        const std::filesystem::path can_config_path = argc == 3 ? argv[2] : "config/ti5/t170c/can.yaml";

        robot::common::logger()->info("TI5 T170C CAN Discovery startup");
        robot::common::logger()->info("Loading robot config {} and CAN config {}",
                                     robot_config_path.string(),
                                     can_config_path.string());

        const auto robot_config = robot::ti5::loadRobotConfig(robot_config_path);
        const auto discovery_options = robot::ti5::loadDiscoveryConfig(can_config_path);

        const robot::ti5::CanDiscovery discovery;
        const auto result = discovery.discover(robot_config.can_buses, discovery_options);
        printDiscoverySummary(result);

        if (!result.success)
        {
            robot::common::logger()->error("TI5 T170C CAN Discovery FAIL");
            return 1;
        }

        robot::common::logger()->info("TI5 T170C CAN Discovery PASS");
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error("TI5 T170C CAN Discovery exited with error: {}", error.what());
        return 1;
    }
    catch (...)
    {
        robot::common::logger()->error("TI5 T170C CAN Discovery exited with unknown error");
        return 1;
    }
}
