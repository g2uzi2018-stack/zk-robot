#include "common/logger.hpp"
#include "ti5/config_loader.hpp"
#include "ti5/discovery.hpp"

#include <exception>
#include <filesystem>

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
