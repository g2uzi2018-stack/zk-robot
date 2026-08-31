#include "input/exoskeleton/exoskeleton.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
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
    frame[kPayloadSize + 1] = calculateChecksum(frame);
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
    writeInt16(frame, 0, 2048);
    writeInt16(frame, 2, 1848);
    frame[1 + 4] = 0x1F;
    frame[1 + 5] = 0x3F;
    writeInt16(frame, 6, 699);

    writeInt16(frame, 8, 2247);
    writeInt16(frame, 10, 2248);
    frame[1 + 12] = 0x3F;
    frame[1 + 13] = 0x1F;
    writeInt16(frame, 14, 3701);

    const std::array<std::int16_t, 7> left_raw{
        0, 16384, -16384, -1, 2, -3, 1234};
    const std::array<std::int16_t, 7> right_raw{
        -16384, 16383, -2, 3, -4, 5, -6};
    for (std::size_t index = 0; index < left_raw.size(); ++index)
    {
        writeInt16(frame, 16 + index * 2, left_raw[index]);
        writeInt16(frame, 32 + index * 2, right_raw[index]);
    }
    // Reserved words are intentionally non-zero. They must not shift either arm.
    writeInt16(frame, 30, 3210);
    writeInt16(frame, 46, -3210);

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
    const double radians_per_count =
        6.283185307179586476925286766559 / 16384.0;
    expect(near(state.left_joint_rad[1], 16384.0 * radians_per_count),
           "left joint little-endian conversion failed");
    expect(near(state.left_joint_rad[2], -16384.0 * radians_per_count),
           "negative left joint conversion failed");
    expect(near(state.right_joint_rad[0], -16384.0 * radians_per_count),
           "right joint offset/reserved word parsing failed");
    expect(state.left.raw_x == 2048 && state.left.raw_y == 1848,
           "left joystick raw values failed");
    expect(near(state.left.x, 0.0) && near(state.left.y, -200.0 / 1848.0),
           "left joystick normalization failed");
    expect(state.right.raw_x == 2247 && near(state.right.x, 0.0),
           "joystick deadzone must use strict less-than boundary");
    expect(near(state.right.y, 200.0 / 1848.0),
           "joystick non-deadzone boundary failed");
    expect(state.left.trigger_raw == 699 && near(state.left.trigger, 0.0),
           "trigger lower boundary failed");
    expect(state.right.trigger_raw == 3701 && near(state.right.trigger, 1.0),
           "trigger upper boundary failed");
    expect(state.left.buttons_raw == 0x1F &&
               state.left.extended_buttons_raw == 0x3F,
           "left raw button bytes were not preserved");
    expect(state.left_mode_raw == 0x1F && state.right_mode_raw == 0x3F,
           "raw mode values must retain 0x1F/0x3F distinction");
    expect(!state.left.buttonPressed(0x01),
           "high button bit must mean released for low-active buttons");
    expect(state.torso_imu.present && state.torso_imu.valid &&
               state.head_imu.present && state.head_imu.valid,
           "valid IMU was rejected");
    expect(near(state.torso_imu.acceleration[1], -2.5) &&
               near(state.torso_imu.quaternion_wxyz[3], 0.5),
           "float32 little-endian parsing failed");
    expect(state.timestamp == std::chrono::steady_clock::time_point{},
           "protocol layer must not assign a host timestamp");
}

void testValidationAndTriggerBoundaries()
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

    for (const auto &[raw, expected] : std::array<std::pair<std::int16_t, double>, 4>{
             std::pair<std::int16_t, double>{699, 0.0},
             std::pair<std::int16_t, double>{700, 0.0},
             std::pair<std::int16_t, double>{3700, 1.0},
             std::pair<std::int16_t, double>{3701, 1.0}})
    {
        auto boundary = validImuFrame();
        writeInt16(boundary, 6, raw);
        finish(boundary);
        const auto state = ExoskeletonProtocol::parse(boundary);
        expect(near(state.left.trigger, expected),
               "trigger boundary mapping failed");
    }

    auto clamped = validImuFrame();
    writeInt16(clamped, 0, std::numeric_limits<std::int16_t>::max());
    writeInt16(clamped, 2, std::numeric_limits<std::int16_t>::min());
    finish(clamped);
    const auto clamped_state = ExoskeletonProtocol::parse(clamped);
    expect(near(clamped_state.left.x, 1.0) &&
               near(clamped_state.left.y, -1.0),
           "joystick normalized output must be clamped");
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

void testConfig()
{
    const auto config = loadExoskeletonConfig(
        std::filesystem::path{EXOSKELETON_SOURCE_DIR} /
        "config/exoskeleton.yaml");
    expect(!config.device.empty() &&
               std::filesystem::path{config.device}.is_absolute(),
           "config device must be a non-empty absolute path");
    expect(config.baudrate == 2000000, "config baudrate mismatch");
    expect(config.stale_timeout == std::chrono::milliseconds{100},
           "config stale timeout mismatch");
    expect(config.reconnect_interval == std::chrono::milliseconds{1000},
           "config reconnect interval mismatch");
}

} // namespace

int main()
{
    try
    {
        testNormalFrameAndFields();
        testValidationAndTriggerBoundaries();
        testInvalidImuDoesNotThrow();
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
