#include "input/exoskeleton/exoskeleton.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

namespace
{

using namespace robot::input::exoskeleton;

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

template <typename Callable>
void expectThrows(Callable &&callable, const std::string &message)
{
    try
    {
        callable();
    }
    catch (const std::exception &)
    {
        return;
    }
    throw std::runtime_error(message);
}

bool near(const double actual, const double expected, const double tolerance = 1e-12)
{
    return std::abs(actual - expected) <= tolerance;
}

void writeInt16(
    ExoskeletonFrame &frame,
    const std::size_t payload_offset,
    const std::int16_t value)
{
    const auto raw = static_cast<std::uint16_t>(value);
    frame[payload_offset + 1] = static_cast<std::uint8_t>(raw & 0xFFU);
    frame[payload_offset + 2] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
}

void writeFloat(
    ExoskeletonFrame &frame,
    const std::size_t payload_offset,
    const float value)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));
    std::uint32_t bits{};
    std::memcpy(&bits, &value, sizeof(bits));
    for (std::size_t index = 0; index < sizeof(bits); ++index)
    {
        frame[payload_offset + 1 + index] =
            static_cast<std::uint8_t>((bits >> (index * 8U)) & 0xFFU);
    }
}

ExoskeletonFrame emptyFrame()
{
    ExoskeletonFrame frame{};
    frame[0] = kFrameHead;
    frame[kFrameSize - 1] = kFrameTail;
    return frame;
}

void finish(ExoskeletonFrame &frame)
{
    frame[frame.size() - 2] = calculateChecksum(frame);
}

ExoskeletonFrame makeLegacyFrame(
    const std::size_t payload_size,
    const std::uint8_t payload_value = 0)
{
    ExoskeletonFrame frame{};
    frame.frame_size = payload_size + 3;
    frame.fill(payload_value);
    frame[0] = kFrameHead;
    frame[frame.size() - 1] = kFrameTail;
    finish(frame);
    return frame;
}

void writeUnitQuaternion(
    ExoskeletonFrame &frame,
    const std::size_t payload_offset)
{
    writeFloat(frame, payload_offset + 24, 1.0F);
    writeFloat(frame, payload_offset + 28, 0.0F);
    writeFloat(frame, payload_offset + 32, 0.0F);
    writeFloat(frame, payload_offset + 36, 0.0F);
}

ExoskeletonFrame validImuFrame()
{
    auto frame = emptyFrame();
    for (const std::size_t offset : {std::size_t{48}, std::size_t{88}})
    {
        for (std::size_t index = 0; index < 10; ++index)
        {
            writeFloat(frame, offset + index * 4, 0.0F);
        }
        writeUnitQuaternion(frame, offset);
    }
    finish(frame);
    return frame;
}

void testNormalFrameAndFields()
{
    auto frame = validImuFrame();
    writeInt16(frame, 0, 1234);
    writeInt16(frame, 2, -567);
    writeInt16(frame, 4, static_cast<std::int16_t>(0x1F3F));
    writeInt16(frame, 6, 512);

    writeInt16(frame, 8, -1234);
    writeInt16(frame, 10, 567);
    writeInt16(frame, 12, 0x001F);
    writeInt16(frame, 14, 3584);

    const std::array<std::int16_t, 8> left_raw{
        0, 16384, -16384, -1, 2, -3, 1234, -2222};
    const std::array<std::int16_t, 8> right_raw{
        -16384, 16383, -2, 3, -4, 5, -6, 3333};
    for (std::size_t index = 0; index < left_raw.size(); ++index)
    {
        writeInt16(frame, 16 + index * 2, left_raw[index]);
        writeInt16(frame, 32 + index * 2, right_raw[index]);
    }

    writeFloat(frame, 48, 1.25F);
    writeFloat(frame, 52, -2.5F);
    writeFloat(frame, 56, 0.25F);
    writeFloat(frame, 60, -4.0F);
    writeFloat(frame, 64, 5.0F);
    writeFloat(frame, 68, 6.0F);
    writeFloat(frame, 72, 0.5F);
    writeFloat(frame, 76, 0.5F);
    writeFloat(frame, 80, 0.5F);
    writeFloat(frame, 84, 0.5F);
    writeFloat(frame, 88, -1.5F);
    writeFloat(frame, 92, 2.5F);
    writeFloat(frame, 96, 3.5F);
    writeFloat(frame, 100, 7.0F);
    writeFloat(frame, 104, 8.0F);
    writeFloat(frame, 108, 9.0F);
    writeFloat(frame, 112, 0.5F);
    writeFloat(frame, 116, 0.5F);
    writeFloat(frame, 120, 0.5F);
    writeFloat(frame, 124, 0.5F);
    finish(frame);

    expect(isValidFrame(frame), "normal frame rejected");
    const auto state = ExoskeletonProtocol::parse(frame);
    const double radians_per_count = kVendorEncoderToRadianRatio;
    const double pi = std::acos(-1.0);
    expect(near(state.left_arm_joint_rad[1], 2.0 * pi) &&
               near(state.left_arm_joint_rad[2], -2.0 * pi),
           "left joint raw-to-radian conversion failed");
    expect(near(state.right_arm_joint_rad[0], -16384.0 * radians_per_count),
           "right joint offset/reserved word parsing failed");
    expect(state.left.raw_x == 1234 && state.left.raw_y == -567,
           "left joystick raw values failed");
    expect(state.right.raw_x == -1234 && state.right.raw_y == 567 &&
               state.left.trigger_raw == 512 &&
               state.right.trigger_raw == 3584,
           "right joystick or trigger raw values failed");
    expect(state.left.key_mask_raw == 0x1F3F &&
               state.right.key_mask_raw == 0x001F,
           "raw key_mask values were not preserved");
    expect(state.left.toggleOn() && !state.right.toggleOn() &&
               state.left.buttonsReleased() &&
               state.left.extendedButtonsReleased(),
           "vendor key_mask bit semantics failed");
    expect(!state.left.buttonPressed(0x01),
           "high button bit must mean released for low-active buttons");
    expect(!state.left.extendedButtonPressed(0x01),
           "high extension button bit must mean released for low-active buttons");
    expect(state.left_arm_joint_raw == left_raw &&
               state.right_arm_joint_raw == right_raw &&
               near(state.left_arm_joint_rad[7],
                    -2222.0 * radians_per_count) &&
               near(state.right_arm_joint_rad[7],
                    3333.0 * radians_per_count),
           "all eight vendor encoder slots were not preserved");
    expect(state.torso_imu.present && state.torso_imu.valid &&
               state.head_imu.present && state.head_imu.valid,
           "valid IMU was rejected");
    expect(near(state.torso_imu.acceleration[1], -2.5) &&
               near(state.torso_imu.quaternion_wxyz[3], 0.5),
           "float32 little-endian parsing failed");
    expect(state.timestamp == std::chrono::steady_clock::time_point{},
           "protocol layer must not assign a host timestamp");
}

void testValidation()
{
    auto frame = validImuFrame();
    expect(ExoskeletonProtocol::validateFrame(frame),
           "valid checksum/head/tail was rejected");

    auto wrong_head = frame;
    wrong_head[0] = 0xAB;
    expect(!isValidFrame(wrong_head), "wrong frame head accepted");
    expectThrows(
        [&] { parseFrame(wrong_head); },
        "wrong frame head must throw");

    auto wrong_tail = frame;
    wrong_tail[kFrameSize - 1] = 0x54;
    expect(!isValidFrame(wrong_tail), "wrong frame tail accepted");
    expectThrows(
        [&] { ExoskeletonProtocol::parse(wrong_tail); },
        "wrong frame tail must throw");

    auto wrong_checksum = frame;
    wrong_checksum[kPayloadSize + 1] ^= 0x01;
    expect(!isValidFrame(wrong_checksum), "wrong checksum accepted");
    expectThrows(
        [&] { ExoskeletonProtocol::parse(wrong_checksum); },
        "wrong checksum must throw");

}

void testInvalidImuDoesNotThrow()
{
    auto zero_quaternion = validImuFrame();
    for (std::size_t index = 0; index < 4; ++index)
    {
        writeFloat(zero_quaternion, 72 + index * 4, 0.0F);
    }
    finish(zero_quaternion);
    const auto zero_state = ExoskeletonProtocol::parse(zero_quaternion);
    expect(zero_state.torso_imu.present && !zero_state.torso_imu.valid,
           "zero quaternion must be invalid");

    auto nan_frame = validImuFrame();
    writeFloat(nan_frame, 48, std::numeric_limits<float>::quiet_NaN());
    finish(nan_frame);
    const auto nan_state = ExoskeletonProtocol::parse(nan_frame);
    expect(!nan_state.torso_imu.valid,
           "NaN IMU field must mark IMU invalid");

    auto infinity_frame = validImuFrame();
    writeFloat(infinity_frame, 60, std::numeric_limits<float>::infinity());
    finish(infinity_frame);
    const auto infinity_state = ExoskeletonProtocol::parse(infinity_frame);
    expect(!infinity_state.torso_imu.valid,
           "infinite IMU field must mark IMU invalid");
}

void testVendorFrameLengths()
{
    auto base = makeLegacyFrame(kLegacyBasePayloadSize);
    writeInt16(base, 0, 2000);
    writeInt16(base, 16, -1234);
    writeInt16(base, 32, 2345);
    finish(base);

    expect(isSupportedFrameSize(base.size()), "51-byte frame size not supported");
    expect(isValidFrame(base), "valid 51-byte vendor frame rejected");
    const auto base_state = parseFrame(base);
    expect(base_state.frame_size == kLegacyBaseFrameSize &&
               base_state.format_version == 1,
           "51-byte vendor format metadata failed");
    expect(!base_state.torso_imu.present && !base_state.head_imu.present,
           "51-byte vendor frame must not invent IMU data");
    expect(base_state.left_arm_joint_rad[0] != 0.0 &&
               base_state.right_arm_joint_rad[0] != 0.0,
           "51-byte vendor arm data failed");

    auto torso = makeLegacyFrame(kLegacyTorsoImuPayloadSize);
    writeFloat(torso, 48, 1.0F);
    writeFloat(torso, 52, 2.0F);
    writeFloat(torso, 56, 3.0F);
    writeUnitQuaternion(torso, 48);
    finish(torso);
    const auto torso_state = parseFrame(torso);
    expect(torso_state.frame_size == kLegacyTorsoImuFrameSize &&
               torso_state.format_version == 2 &&
               torso_state.torso_imu.present && torso_state.torso_imu.valid &&
               !torso_state.head_imu.present,
           "91-byte vendor format was not decoded as torso-only IMU");

    auto full = makeLegacyFrame(kLegacyFullPayloadSize);
    writeUnitQuaternion(full, 48);
    writeUnitQuaternion(full, 88);
    finish(full);
    const auto full_state = parseFrame(full);
    expect(full_state.frame_size == kLegacyFullFrameSize &&
               full_state.format_version == 3 &&
               full_state.torso_imu.present && full_state.head_imu.present,
           "131-byte vendor format metadata failed");
}

void testConfig()
{
    const auto unique_suffix = std::to_string(::getpid()) + "_" +
                               std::to_string(
                                   std::chrono::steady_clock::now()
                                       .time_since_epoch()
                                       .count());
    const auto config_path = std::filesystem::temp_directory_path() /
                             ("zk_robot_exoskeleton_" + unique_suffix +
                              ".yaml");
    {
        std::ofstream output(config_path);
        expect(output.good(), "failed to create temporary config");
        output << "exoskeleton:\n"
               << "  serial:\n"
               << "    device: auto\n"
               << "    usb_vid: 0x0483\n"
               << "    usb_pid: 0x5740\n"
               << "    match_vid_only: false\n"
               << "    baudrate: 2000000\n"
               << "    poll_timeout_ms: 20\n"
               << "    reconnect_interval_ms: 1000\n"
               << "  telemetry:\n"
               << "    frame_size: 131\n"
               << "    stale_timeout_ms: 100\n";
    }

    const auto config = loadExoskeletonConfig(config_path);
    std::error_code remove_error;
    std::filesystem::remove(config_path, remove_error);
    expect(config.usb_vid == 0x0483, "config USB VID mismatch");
    expect(config.usb_pid == 0x5740, "config USB PID mismatch");
    expect(!config.match_vid_only, "config must use strict VID:PID matching");
    expect(config.device == "auto", "config device mismatch");
    expect(config.baudrate == 2000000, "config baudrate mismatch");
    expect(config.poll_timeout == std::chrono::milliseconds{20},
           "config poll timeout mismatch");
    expect(config.stale_timeout == std::chrono::milliseconds{100},
           "config stale timeout mismatch");
    expect(config.reconnect_interval == std::chrono::milliseconds{1000},
           "config reconnect interval mismatch");
    expect(config.frame_mode == ExoskeletonFrameMode::Full,
           "config frame mode mismatch");
}

} // namespace

int main()
{
    try
    {
        testNormalFrameAndFields();
        testValidation();
        testInvalidImuDoesNotThrow();
        testVendorFrameLengths();
        testConfig();
        std::cout << "Exoskeleton protocol tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Exoskeleton protocol test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
