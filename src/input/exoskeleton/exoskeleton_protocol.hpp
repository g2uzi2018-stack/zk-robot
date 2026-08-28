#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace robot::input::exoskeleton
{

constexpr std::size_t kFrameSize = 131;
constexpr std::size_t kPayloadSize = 128;
constexpr std::uint8_t kFrameHead = 0xAA;
constexpr std::uint8_t kFrameTail = 0x55;

using ExoskeletonFrame = std::array<std::uint8_t, kFrameSize>;

struct JoystickState
{
    std::int16_t raw_x{0};
    std::int16_t raw_y{0};

    double x{0.0};
    double y{0.0};

    std::int16_t trigger_raw{0};
    double trigger{0.0};

    std::uint8_t buttons_raw{0};
    std::uint8_t extended_buttons_raw{0};

    // 按键低电平有效；mask 通常使用低 5 位中的单 bit 掩码。
    bool buttonPressed(const std::uint8_t mask) const noexcept
    {
        return mask != 0 && (buttons_raw & mask) == 0;
    }

    bool extendedButtonPressed(const std::uint8_t mask) const noexcept
    {
        return mask != 0 && (extended_buttons_raw & mask) == 0;
    }
};

struct ImuState
{
    std::array<float, 3> acceleration{};
    std::array<float, 3> angular_velocity_raw{};
    std::array<float, 4> quaternion_wxyz{};

    bool present{false};
    bool valid{false};
};

struct ExoskeletonState
{
    std::array<double, 7> left_joint_rad{};
    std::array<double, 7> right_joint_rad{};

    JoystickState left;
    JoystickState right;

    ImuState torso_imu;
    ImuState head_imu;

    // 这是普通按键字节的完整原始值，不能先截断为低 5 bit；
    // 0x1F/0x3F 等档位语义留给上层状态机解释。
    std::uint8_t left_mode_raw{0};
    std::uint8_t right_mode_raw{0};

    // Protocol 层不填写它；设备层在收到完整合法帧后写入主机单调时钟。
    std::chrono::steady_clock::time_point timestamp{};
};

class ExoskeletonProtocol final
{
public:
    using Frame = ExoskeletonFrame;

    static std::uint8_t calculateChecksum(const ExoskeletonFrame &frame) noexcept;
    static bool validateFrame(const ExoskeletonFrame &frame) noexcept;
    static ExoskeletonState parse(const ExoskeletonFrame &frame);
};

// 便于不需要保存无状态 parser 对象的调用方使用。
std::uint8_t calculateChecksum(const ExoskeletonFrame &frame) noexcept;
bool isValidFrame(const ExoskeletonFrame &frame) noexcept;
ExoskeletonState parseFrame(const ExoskeletonFrame &frame);

} // namespace robot::input::exoskeleton
