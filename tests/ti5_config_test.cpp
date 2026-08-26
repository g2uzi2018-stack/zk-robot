#include "ti5/config/config_loader.hpp"
#include "ti5/hand/hand_config.hpp"
#include "ti5/joint/joint_config_builder.hpp"

#include <cmath>
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
        const auto safety = robot::ti5::loadJointSafetyConfig(
            source_root / "config/ti5/t170c/safety.yaml");
        const auto kinematics = robot::ti5::loadKinematicsConfig(
            source_root / "config/ti5/t170c/kinematics.yaml");
        const auto hands = robot::ti5::hand::loadHandConfig(
            source_root / "config/ti5/t170c/hands.yaml");

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
        expect(safety.position_limits.size() == 22 &&
                   safety.position_limits.at("left_shoulder_roll").minimum_rad <
                       -1.58 &&
                   !safety.position_limits.at("left_shoulder_roll")
                        .verified_on_robot,
               "Joint software position limits were not fully loaded");
        expect(kinematics.models.size() == 3 &&
                   kinematics.models.at("t7_t170_left_arm").joints.size() == 7 &&
                   kinematics.models.at("t7_t170_right_arm").joints.size() == 7 &&
                   kinematics.models.at("folded_leg_3r").joints.size() == 3,
               "Joint kinematics model sizes mismatch");
        const auto &left_pitch = kinematics.models
                                     .at("t7_t170_left_arm")
                                     .joints.at("left_shoulder_pitch");
        const auto &left_roll = kinematics.models
                                    .at("t7_t170_left_arm")
                                    .joints.at("left_shoulder_roll");
        expect(left_pitch.direction == -1.0 &&
                   std::abs(left_roll.offset_rad +
                            1.5707963267948966) < 1e-12,
               "Joint coordinate direction or zero offset mismatch");

        const auto left_arm_configs = robot::ti5::makeJointConfigs(
            robot,
            safety,
            &kinematics.models.at("t7_t170_left_arm"));
        expect(left_arm_configs.size() == 7 &&
                   left_arm_configs.front().physical_joint.name ==
                       "left_shoulder_pitch" &&
                   left_arm_configs.front().coordinate_transform.direction ==
                       -1.0,
               "JointConfig model assembly mismatch");
        const auto all_identity_configs = robot::ti5::makeJointConfigs(
            robot, safety);
        expect(all_identity_configs.size() == 22 &&
                   all_identity_configs.at(5).physical_joint.name ==
                       "neck_yaw" &&
                   all_identity_configs.at(5)
                           .coordinate_transform.direction == 1.0,
               "identity JointConfig assembly mismatch");

        expect(hands.left.controller_node_id == 70 &&
                   hands.right.controller_node_id == 60 &&
                   !hands.left.protocol_verified &&
                   !hands.left.control_enabled &&
                   hands.transport.bitrate == 1000000,
               "Aoyi hand config or default control guard mismatch");

        std::cout << "TI5 config tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "TI5 config test failed: " << error.what() << '\n';
        return 1;
    }
}
