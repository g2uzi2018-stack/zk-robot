#include "input/exoskeleton/exoskeleton_protocol.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace
{

constexpr std::size_t kLeftJoystickXOffset = 0;
constexpr std::size_t kLeftJoystickYOffset = 2;
constexpr std::size_t kLeftButtonsOffset = 4;
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

robot::input::exoskeleton::JoystickState parseJoystick(
    const robot::input::exoskeleton::ExoskeletonFrame &frame,
    const std::size_t offset)
{
    robot::input::exoskeleton::JoystickState result;
    result.raw_x = readSigned16(frame, offset + kLeftJoystickXOffset);
    result.raw_y = readSigned16(frame, offset + kLeftJoystickYOffset);
    result.key_mask_raw = readUnsigned16(
        frame,
        offset + kLeftButtonsOffset);
    result.trigger_raw = readSigned16(frame, offset + kLeftTriggerOffset);
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

bool isSupportedFrameSize(const std::size_t frame_size) noexcept
{
    return frame_size == kLegacyBaseFrameSize ||
           frame_size == kLegacyTorsoImuFrameSize ||
           frame_size == kLegacyFullFrameSize;
}

std::uint8_t ExoskeletonProtocol::calculateChecksum(
    const ExoskeletonFrame &frame) noexcept
{
    if (!isSupportedFrameSize(frame.frame_size))
    {
        return 0;
    }

    std::uint8_t checksum = 0;
    for (std::size_t index = 1; index + 2 < frame.frame_size; ++index)
    {
        checksum = static_cast<std::uint8_t>(checksum ^ frame[index]);
    }
    return checksum;
}

bool ExoskeletonProtocol::validateFrame(
    const ExoskeletonFrame &frame) noexcept
{
    if (!isSupportedFrameSize(frame.frame_size))
    {
        return false;
    }

    return frame[0] == kFrameHead &&
           frame[frame.frame_size - 1] == kFrameTail &&
           calculateChecksum(frame) == frame[frame.frame_size - 2];
}

ExoskeletonState ExoskeletonProtocol::parse(const ExoskeletonFrame &frame)
{
    if (!isSupportedFrameSize(frame.frame_size))
    {
        throw std::invalid_argument("Invalid exoskeleton frame size");
    }
    if (frame[0] != kFrameHead)
    {
        throw std::invalid_argument("Invalid exoskeleton frame head");
    }
    if (frame[frame.frame_size - 1] != kFrameTail)
    {
        throw std::invalid_argument("Invalid exoskeleton frame tail");
    }
    if (calculateChecksum(frame) != frame[frame.frame_size - 2])
    {
        throw std::invalid_argument("Invalid exoskeleton frame checksum");
    }

    ExoskeletonState result;
    result.frame_size = frame.frame_size;
    result.format_version = 1;
    result.left = parseJoystick(frame, 0);
    result.right = parseJoystick(frame, 8);

    for (std::size_t index = 0; index < kVendorArmEncoderCount; ++index)
    {
        result.left_arm_joint_raw[index] =
            readSigned16(frame, kLeftJointsOffset + index * 2);
        result.right_arm_joint_raw[index] =
            readSigned16(frame, kRightJointsOffset + index * 2);
        result.left_arm_joint_rad[index] =
            static_cast<double>(result.left_arm_joint_raw[index]) *
            kVendorEncoderToRadianRatio;
        result.right_arm_joint_rad[index] =
            static_cast<double>(result.right_arm_joint_raw[index]) *
            kVendorEncoderToRadianRatio;
    }
    if (frame.frame_size - 3 >= kLegacyTorsoImuPayloadSize)
    {
        result.torso_imu = parseImu(frame, kTorsoImuOffset);
        result.format_version = 2;
    }
    if (frame.frame_size - 3 >= kLegacyFullPayloadSize)
    {
        result.head_imu = parseImu(frame, kHeadImuOffset);
        result.format_version = 3;
    }

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
