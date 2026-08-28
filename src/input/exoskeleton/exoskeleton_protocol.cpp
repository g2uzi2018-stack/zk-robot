#include "input/exoskeleton/exoskeleton_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace
{

constexpr std::size_t kLeftJoystickXOffset = 0;
constexpr std::size_t kLeftJoystickYOffset = 2;
constexpr std::size_t kLeftButtonsOffset = 4;
constexpr std::size_t kLeftExtendedButtonsOffset = 5;
constexpr std::size_t kLeftTriggerOffset = 6;

constexpr std::size_t kRightJoystickXOffset = 8;
constexpr std::size_t kRightJoystickYOffset = 10;
constexpr std::size_t kRightButtonsOffset = 12;
constexpr std::size_t kRightExtendedButtonsOffset = 13;
constexpr std::size_t kRightTriggerOffset = 14;

constexpr std::size_t kLeftJointsOffset = 16;
constexpr std::size_t kRightJointsOffset = 32;
constexpr std::size_t kTorsoImuOffset = 48;
constexpr std::size_t kHeadImuOffset = 88;

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr std::int16_t kJoystickCenter = 2048;
constexpr std::int16_t kJoystickDeadzone = 200;
constexpr double kJoystickScale = 1848.0;

std::uint16_t readUnsigned16(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t payload_offset)
{
    const std::size_t offset = payload_offset + 1;
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(frame[offset]) |
        (static_cast<std::uint16_t>(frame[offset + 1]) << 8U));
}

std::int16_t readSigned16(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t payload_offset)
{
    const std::uint16_t raw = readUnsigned16(frame, payload_offset);
    const std::int32_t value = (raw & 0x8000U) != 0
                                   ? static_cast<std::int32_t>(raw) - 0x10000
                                   : static_cast<std::int32_t>(raw);
    return static_cast<std::int16_t>(value);
}

float readFloat32(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t payload_offset)
{
    static_assert(sizeof(float) == sizeof(std::uint32_t));

    const std::size_t offset = payload_offset + 1;
    const std::uint32_t bits =
        static_cast<std::uint32_t>(frame[offset]) |
        (static_cast<std::uint32_t>(frame[offset + 1]) << 8U) |
        (static_cast<std::uint32_t>(frame[offset + 2]) << 16U) |
        (static_cast<std::uint32_t>(frame[offset + 3]) << 24U);

    float value{};
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

double normalizeJoystick(const std::int16_t raw)
{
    const double difference =
        static_cast<double>(raw) - static_cast<double>(kJoystickCenter);
    if (std::abs(difference) < static_cast<double>(kJoystickDeadzone))
    {
        return 0.0;
    }

    return std::clamp(difference / kJoystickScale, -1.0, 1.0);
}

double normalizeTrigger(const std::int16_t raw)
{
    if (raw < 700)
    {
        return 0.0;
    }
    if (raw > 3700)
    {
        return 1.0;
    }
    return std::clamp(
        (static_cast<double>(raw) - 700.0) / 3000.0,
        0.0,
        1.0);
}

robot::input::exoskeleton::JoystickState parseJoystick(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t offset)
{
    robot::input::exoskeleton::JoystickState result;
    result.raw_x = readSigned16(frame, offset + kLeftJoystickXOffset);
    result.raw_y = readSigned16(frame, offset + kLeftJoystickYOffset);
    result.x = normalizeJoystick(result.raw_x);
    result.y = normalizeJoystick(result.raw_y);
    result.buttons_raw = frame[offset + kLeftButtonsOffset + 1];
    result.extended_buttons_raw =
        frame[offset + kLeftExtendedButtonsOffset + 1];
    result.trigger_raw = readSigned16(frame, offset + kLeftTriggerOffset);
    result.trigger = normalizeTrigger(result.trigger_raw);
    return result;
}

robot::input::exoskeleton::ImuState parseImu(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t offset)
{
    robot::input::exoskeleton::ImuState result;
    result.present = true;

    for (std::size_t index = 0; index < result.acceleration.size(); ++index)
    {
        result.acceleration[index] = readFloat32(frame, offset + index * 4);
        result.angular_velocity_raw[index] =
            readFloat32(frame, offset + 12 + index * 4);
    }
    for (std::size_t index = 0; index < result.quaternion_wxyz.size(); ++index)
    {
        result.quaternion_wxyz[index] =
            readFloat32(frame, offset + 24 + index * 4);
    }

    result.valid = true;
    for (const float value : result.acceleration)
    {
        result.valid = result.valid && std::isfinite(value);
    }
    for (const float value : result.angular_velocity_raw)
    {
        result.valid = result.valid && std::isfinite(value);
    }
    for (const float value : result.quaternion_wxyz)
    {
        result.valid = result.valid && std::isfinite(value);
    }

    if (!result.valid)
    {
        return result;
    }

    double norm_squared = 0.0;
    for (const float value : result.quaternion_wxyz)
    {
        norm_squared += static_cast<double>(value) * value;
    }
    const double norm = std::sqrt(norm_squared);
    // 不擅自归一化 payload；只接受明显接近单位四元数的值。
    result.valid = std::isfinite(norm) && norm >= 0.5 && norm <= 1.5;
    return result;
}

} // namespace

namespace robot::input::exoskeleton
{

std::uint8_t ExoskeletonProtocol::calculateChecksum(
    const ExoskeletonFrame &frame) noexcept
{
    std::uint8_t checksum = 0;
    for (std::size_t index = 1; index <= kPayloadSize; ++index)
    {
        checksum = static_cast<std::uint8_t>(checksum ^ frame[index]);
    }
    return checksum;
}

bool ExoskeletonProtocol::validateFrame(
    const ExoskeletonFrame &frame) noexcept
{
    return frame[0] == kFrameHead &&
           frame[kFrameSize - 1] == kFrameTail &&
           calculateChecksum(frame) == frame[kPayloadSize + 1];
}

ExoskeletonState ExoskeletonProtocol::parse(const ExoskeletonFrame &frame)
{
    if (frame[0] != kFrameHead)
    {
        throw std::invalid_argument("Invalid exoskeleton frame head");
    }
    if (frame[kFrameSize - 1] != kFrameTail)
    {
        throw std::invalid_argument("Invalid exoskeleton frame tail");
    }
    if (calculateChecksum(frame) != frame[kPayloadSize + 1])
    {
        throw std::invalid_argument("Invalid exoskeleton frame checksum");
    }

    ExoskeletonState result;
    result.left = parseJoystick(frame, 0);
    result.right = parseJoystick(frame, 8);

    for (std::size_t index = 0; index < result.left_joint_rad.size(); ++index)
    {
        result.left_joint_rad[index] =
            static_cast<double>(readSigned16(frame, kLeftJointsOffset + index * 2)) *
            (kTwoPi / 16384.0);
        result.right_joint_rad[index] =
            static_cast<double>(readSigned16(frame, kRightJointsOffset + index * 2)) *
            (kTwoPi / 16384.0);
    }

    result.torso_imu = parseImu(frame, kTorsoImuOffset);
    result.head_imu = parseImu(frame, kHeadImuOffset);

    // 模式字节就是普通 buttons 字节的完整 raw 值；高位不能丢失。
    result.left_mode_raw = result.left.buttons_raw;
    result.right_mode_raw = result.right.buttons_raw;
    return result;
}

std::uint8_t calculateChecksum(const ExoskeletonFrame &frame) noexcept
{
    return ExoskeletonProtocol::calculateChecksum(frame);
}

bool isValidFrame(const ExoskeletonFrame &frame) noexcept
{
    return ExoskeletonProtocol::validateFrame(frame);
}

ExoskeletonState parseFrame(const ExoskeletonFrame &frame)
{
    return ExoskeletonProtocol::parse(frame);
}

} // namespace robot::input::exoskeleton
