#include "ti5/config/config_loader.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    try
    {
        const auto source_root = std::filesystem::path{TI5_SOURCE_DIR};
        const auto robot = robot::ti5::loadRobotConfig(
            source_root / "config/ti5/t170c/robot.yaml");
        const auto can = robot::ti5::loadCanConfig(
            source_root / "config/ti5/t170c/can.yaml");

        expect(robot.vendor == "TI5" && robot.model == "T170C" &&
                   robot.body_motor_count == 22 && robot.joints.size() == 22,
               "T170C robot topology config mismatch");
        expect(robot.can_buses.size() == 4,
               "T170C must contain four body logical buses");
        expect(can.socketcan.bitrate == 1000000 &&
                   can.socketcan.body_adapter.expected_channels == 4,
               "SocketCAN adapter config mismatch");
        expect(can.discovery.enabled && !can.discovery.cache_mapping &&
                   can.discovery.strategy == "expected_node_ids" &&
                   !can.discovery.discover_hands,
               "CAN discovery config was not fully loaded");
        expect(can.receive.centralized_receiver &&
                   can.receive.latest_feedback_cache &&
                   can.receive.use_can_filters &&
                   can.receive.receive_error_frames &&
                   can.receive.timestamp_clock == "monotonic",
               "CAN receive config was not fully loaded");
        expect(can.control.frequency_hz == 100 &&
                   can.control.inter_frame_gap.count() == 50 &&
                   can.control.post_batch_feedback_wait.count() == 200 &&
                   can.control.send_failure_threshold == 30,
               "CAN control config was not fully loaded");
        expect(can.watchdog.stale_feedback_cycles == 3 &&
                   can.watchdog.reject_new_motion_on_stale_feedback &&
                   can.watchdog.enter_fault_on_stale_feedback &&
                   can.watchdog.enter_fault_on_bus_off,
               "CAN watchdog config was not fully loaded");
        expect(can.exclusive_control.enabled &&
                   can.exclusive_control.reject_second_controller &&
                   can.exclusive_control.lock_file ==
                       "/run/lock/ti5-can-controller.lock",
               "exclusive controller config was not fully loaded");

        std::cout << "TI5 config tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 config test failed: " << error.what() << '\n';
        return 1;
    }
}
