#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/hand/hand_config.hpp"
#include "ti5/hand/hand_discovery.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string numberOrUnknown(const std::optional<std::uint32_t> &value)
{
    return value ? std::to_string(*value) : "unknown";
}

std::string stringOrUnknown(const std::optional<std::string> &value)
{
    return value && !value->empty() ? *value : "unknown";
}

} // namespace

int main()
{
    try
    {
        robot::common::logger()->info("Robot startup");
        robot::common::logger()->info("TI5 T170C CAN initialization test");

        const std::filesystem::path robot_config_path{
            "config/ti5/t170c/robot.yaml"};

        const std::filesystem::path can_config_path{
            "config/ti5/t170c/can.yaml"};

        // ============================================================
        // 1. Load configuration
        // ============================================================

        const auto robot_config =
            robot::ti5::loadRobotConfig(robot_config_path);

        const auto can_config =
            robot::ti5::loadCanConfig(can_config_path);

        const auto discovery_options =
            robot::ti5::makeDiscoveryOptions(can_config);

        // ============================================================
        // 2. Enumerate all Linux CAN interfaces
        // ============================================================

        const robot::can::CanInterfaceManager interface_manager;

        const auto all_interfaces =
            interface_manager.enumerate(
                can_config.socketcan.interface_regex);

        if (all_interfaces.empty())
        {
            throw std::runtime_error(
                "No SocketCAN interfaces found");
        }

        robot::common::logger()->info(
            "Detected {} SocketCAN interfaces",
            all_interfaces.size());

        for (const auto &interface : all_interfaces)
        {
            robot::common::logger()->info(
                "CAN {}: state={}, bitrate={}, restart-ms={}, "
                "ID_PATH={}, ID_SERIAL_SHORT={}",
                interface.name,
                interface.up ? "UP" : "DOWN",
                numberOrUnknown(interface.bitrate),
                numberOrUnknown(interface.restart_ms),
                stringOrUnknown(interface.id_path),
                stringOrUnknown(interface.usb_serial_short));
        }

        // ============================================================
        // 3. Select the physical USB-CAN adapter used by TI5 body
        // ============================================================

        const auto &selector =
            can_config.socketcan.body_adapter;

        const auto body_interfaces =
            interface_manager.selectAdapter(
                all_interfaces,
                robot::can::CanAdapterSelector{
                    selector.selector,
                    selector.value,
                    selector.expected_channels});

        robot::common::logger()->info(
            "Selected {} TI5 body CAN interfaces using {}={}",
            body_interfaces.size(),
            selector.selector,
            selector.value);

        // ============================================================
        // 4. Prepare body CAN interfaces
        // ============================================================

        const robot::can::CanInterfaceSettings settings{
            can_config.socketcan.bitrate,
            static_cast<std::uint32_t>(
                can_config.socketcan.restart_ms.count()),
            can_config.socketcan.reconfigure_wait,
            can_config.socketcan.startup_wait,
            can_config.socketcan.validate_bitrate};

        std::vector<std::string> candidate_interfaces;
        candidate_interfaces.reserve(body_interfaces.size());

        for (const auto &interface : body_interfaces)
        {
            robot::common::logger()->info(
                "Preparing body CAN interface {}",
                interface.name);

            robot::can::CanInterfaceInfo ready;

            if (can_config.socketcan.manage_linux_link)
            {
                ready =
                    interface_manager.prepare(
                        interface.name,
                        settings);
            }
            else
            {
                ready =
                    interface_manager.inspect(
                        interface.name);

                if (can_config.socketcan.require_interface_up &&
                    !ready.up)
                {
                    throw std::runtime_error(
                        ready.name + " is not UP");
                }

                if (can_config.socketcan.validate_bitrate &&
                    (!ready.bitrate ||
                     *ready.bitrate != can_config.socketcan.bitrate))
                {
                    throw std::runtime_error(
                        ready.name + " bitrate mismatch");
                }
            }

            robot::common::logger()->info(
                "Body CAN {} ready: state={}, bitrate={}, restart-ms={}",
                ready.name,
                ready.up ? "UP" : "DOWN",
                numberOrUnknown(ready.bitrate),
                numberOrUnknown(ready.restart_ms));

            candidate_interfaces.push_back(interface.name);
        }

        // ============================================================
        // 5. T170C body discovery
        // ============================================================

        robot::common::logger()->info(
            "Starting TI5 body motor discovery");

        const robot::ti5::CanDiscovery discovery;

        const auto result =
            discovery.discover(
                robot_config.can_buses,
                discovery_options,
                candidate_interfaces);

        if (!result.success)
        {
            throw std::runtime_error(
                "TI5 body CAN discovery failed");
        }

        // ============================================================
        // 6. Print final mapping
        // ============================================================

        robot::common::logger()->info(
            "====================================");

        robot::common::logger()->info(
            "TI5 CAN DISCOVERY RESULT");

        robot::common::logger()->info(
            "====================================");

        for (const auto &bus : result.logical_buses)
        {
            if (bus.complete && bus.interface_name)
            {
                robot::common::logger()->info(
                    "{} -> {}",
                    bus.bus_name,
                    *bus.interface_name);
            }
            else
            {
                robot::common::logger()->error(
                    "{} -> NOT FOUND",
                    bus.bus_name);
            }
        }

        robot::common::logger()->info(
            "====================================");

        robot::common::logger()->info(
            "TI5 T170C CAN Discovery PASS");

        // ============================================================
        // 7. Prepare the independent Aoyi hand adapter and discover
        //    left/right ownership by protocol response.
        // ============================================================

        try
        {
            const auto hand_config =
                robot::ti5::hand::loadHandConfig(
                    "config/ti5/t170c/hands.yaml");
            const auto &hand_selector =
                hand_config.transport.adapter_selector;
            const auto hand_interfaces =
                interface_manager.selectAdapter(
                    all_interfaces,
                    hand_selector);

            robot::common::logger()->info(
                "Selected {} Aoyi hand CAN interfaces using {}={}",
                hand_interfaces.size(),
                hand_selector.kind,
                hand_selector.value);

            const robot::can::CanInterfaceSettings hand_settings{
                hand_config.transport.bitrate,
                hand_config.transport.restart_ms,
                hand_config.transport.reconfigure_wait,
                hand_config.transport.startup_wait,
                hand_config.transport.validate_bitrate};

            std::vector<std::string> hand_candidate_interfaces;
            hand_candidate_interfaces.reserve(hand_interfaces.size());
            for (const auto &interface : hand_interfaces)
            {
                robot::can::CanInterfaceInfo ready;
                if (hand_config.transport.manage_linux_link)
                {
                    ready = interface_manager.prepare(
                        interface.name,
                        hand_settings);
                }
                else
                {
                    ready = interface_manager.inspect(interface.name);
                    if (hand_config.transport.require_interface_up &&
                        !ready.up)
                    {
                        throw std::runtime_error(
                            ready.name + " is not UP");
                    }
                    if (hand_config.transport.validate_bitrate &&
                        (!ready.bitrate ||
                         *ready.bitrate !=
                             hand_config.transport.bitrate))
                    {
                        throw std::runtime_error(
                            ready.name + " bitrate mismatch");
                    }
                }

                robot::common::logger()->info(
                    "Aoyi hand CAN {} ready: state={}, bitrate={}, "
                    "restart-ms={}",
                    ready.name,
                    ready.up ? "UP" : "DOWN",
                    numberOrUnknown(ready.bitrate),
                    numberOrUnknown(ready.restart_ms));
                hand_candidate_interfaces.push_back(interface.name);
            }

            const robot::ti5::hand::HandDiscovery hand_discovery;
            const auto hand_result =
                hand_discovery.discover(
                    hand_config,
                    hand_candidate_interfaces);

            robot::common::logger()->info(
                "====================================");
            robot::common::logger()->info(
                "AOYI HAND DISCOVERY RESULT");
            robot::common::logger()->info(
                "====================================");
            robot::common::logger()->info(
                "left_hand -> {}",
                stringOrUnknown(hand_result.left_interface));
            robot::common::logger()->info(
                "right_hand -> {}",
                stringOrUnknown(hand_result.right_interface));
            for (const auto &error : hand_result.errors)
            {
                robot::common::logger()->error("{}", error);
            }

            if (hand_result.success)
            {
                robot::common::logger()->info(
                    "Aoyi Hand Discovery PASS");
            }
            else
            {
                robot::common::logger()->error(
                    "Aoyi Hand Discovery FAIL");
            }
        }
        catch (const std::exception &error)
        {
            robot::common::logger()->error(
                "Aoyi Hand Discovery FAIL: {}",
                error.what());
        }

        robot::common::logger()->info(
            "Robot shutdown");

        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "Robot startup failed: {}",
            error.what());

        return 1;
    }
    catch (...)
    {
        robot::common::logger()->error(
            "Robot startup failed with unknown error");

        return 1;
    }
}
