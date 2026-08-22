#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/config/config_loader.hpp"

#include <exception>
#include <cstdint>
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

std::string optionalOrUnknown(const std::optional<std::string> &value)
{
    return value && !value->empty() ? *value : "unknown";
}

void logInterface(const robot::can::CanInterfaceInfo &interface,
                  const bool selected)
{
    robot::common::logger()->info(
        "CAN interface {} ifindex={} state={} bitrate={} restart-ms={} selected_body={} device_path={} usb_parent={} ID_PATH={} ID_SERIAL_SHORT={} ID_SERIAL={}",
        interface.name,
        interface.ifindex,
        interface.up ? "UP" : "DOWN",
        numberOrUnknown(interface.bitrate),
        numberOrUnknown(interface.restart_ms),
        selected ? "yes" : "no",
        interface.device_path.empty() ? "unknown" : interface.device_path,
        interface.usb_parent.empty() ? "unknown" : interface.usb_parent,
        optionalOrUnknown(interface.id_path),
        optionalOrUnknown(interface.usb_serial_short),
        optionalOrUnknown(interface.id_serial));
}

bool isBodyInterface(const std::set<std::string> &body_names,
                    const std::string &name)
{
    return body_names.find(name) != body_names.end();
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        if (argc != 1 && argc != 3)
        {
            robot::common::logger()->error(
                "Usage: {} [robot.yaml can.yaml]",
                argv[0]);
            return 2;
        }

        const std::filesystem::path robot_config_path =
            argc == 3 ? argv[1] : "config/ti5/t170c/robot.yaml";
        const std::filesystem::path can_config_path =
            argc == 3 ? argv[2] : "config/ti5/t170c/can.yaml";

        robot::common::logger()->info("TI5 T170C CAN Discovery startup");
        robot::common::logger()->info(
            "Loading robot config {} and CAN config {}",
            robot_config_path.string(),
            can_config_path.string());

        const auto robot_config = robot::ti5::loadRobotConfig(robot_config_path);
        const auto can_config = robot::ti5::loadCanConfig(can_config_path);
        const auto discovery_options = robot::ti5::makeDiscoveryOptions(can_config);

        const robot::can::CanInterfaceManager interface_manager;
        const auto all_interfaces = interface_manager.enumerate(
            can_config.socketcan.interface_regex);
        if (all_interfaces.empty())
        {
            throw std::runtime_error(
                "找不到符合 interface_regex 的 SocketCAN 接口");
        }

        const auto &selector_config = can_config.socketcan.body_adapter;
        const auto body_interfaces = interface_manager.selectAdapter(
            all_interfaces,
            robot::can::CanAdapterSelector{
                selector_config.selector,
                selector_config.value,
                selector_config.expected_channels});
        const std::set<std::string> body_names = [&body_interfaces]() {
            std::set<std::string> names;
            for (const auto &interface : body_interfaces)
            {
                names.insert(interface.name);
            }
            return names;
        }();

        for (const auto &interface : all_interfaces)
        {
            logInterface(interface, isBodyInterface(body_names, interface.name));
            if (!isBodyInterface(body_names, interface.name))
            {
                robot::common::logger()->warn(
                    "Ignoring non-body CAN interface {} during T170C discovery",
                    interface.name);
            }
        }

        robot::common::logger()->info(
            "Selected {} body CAN interfaces using {}={}",
            body_interfaces.size(),
            selector_config.selector,
            selector_config.value);

        const robot::can::CanInterfaceSettings settings{
            can_config.socketcan.bitrate,
            static_cast<std::uint32_t>(can_config.socketcan.restart_ms.count()),
            can_config.socketcan.reconfigure_wait,
            can_config.socketcan.startup_wait,
            can_config.socketcan.validate_bitrate};

        std::vector<std::string> candidate_interfaces;
        candidate_interfaces.reserve(body_interfaces.size());
        for (const auto &interface : body_interfaces)
        {
            const bool bitrate_already_correct =
                !can_config.socketcan.validate_bitrate ||
                (interface.bitrate &&
                 *interface.bitrate == can_config.socketcan.bitrate);
            const bool restart_already_correct =
                interface.restart_ms &&
                *interface.restart_ms == settings.restart_ms;
            if (interface.up && bitrate_already_correct && restart_already_correct)
            {
                robot::common::logger()->debug(
                    "Body CAN interface {} is already UP with requested bitrate/restart-ms; no reconfiguration needed",
                    interface.name);
            }

            if (can_config.socketcan.manage_linux_link)
            {
                const auto prepared = interface_manager.prepare(interface.name, settings);
                robot::common::logger()->info(
                    "Body CAN interface {} prepared successfully: state=UP bitrate={} restart-ms={}",
                    prepared.name,
                    numberOrUnknown(prepared.bitrate),
                    numberOrUnknown(prepared.restart_ms));
            }
            else
            {
                const auto current = interface_manager.inspect(interface.name);
                if (can_config.socketcan.require_interface_up && !current.up)
                {
                    throw std::runtime_error(
                        current.name + " 必须处于 UP 状态，但 manage_linux_link=false");
                }
                if (can_config.socketcan.validate_bitrate &&
                    (!current.bitrate ||
                     *current.bitrate != can_config.socketcan.bitrate))
                {
                    throw std::runtime_error(
                        current.name + " bitrate 校验失败，且 manage_linux_link=false");
                }
                if (!current.restart_ms ||
                    *current.restart_ms != settings.restart_ms)
                {
                    throw std::runtime_error(
                        current.name + " restart-ms 校验失败，且 manage_linux_link=false");
                }
            }
            candidate_interfaces.push_back(interface.name);
        }

        robot::common::logger()->info(
            "Running T170C Body Discovery only on {} selected interfaces",
            candidate_interfaces.size());
        const robot::ti5::CanDiscovery discovery;
        const auto result = discovery.discover(
            robot_config.can_buses,
            discovery_options,
            candidate_interfaces);

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
        robot::common::logger()->error(
            "TI5 T170C CAN Discovery exited with error: {}",
            error.what());
        return 1;
    }
    catch (...)
    {
        robot::common::logger()->error(
            "TI5 T170C CAN Discovery exited with unknown error");
        return 1;
    }
}
