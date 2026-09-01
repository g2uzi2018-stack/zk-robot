#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace robot::input::exoskeleton
{

// Qnbot SDK v1.2 的 legacy 遥测流支持三种载荷格式。kFrameSize 和
// kPayloadSize 保留为当前设备的最大格式，ExoskeletonFrame::frame_size
// 才是某一帧实际使用的长度。
constexpr std::size_t kLegacyBasePayloadSize = 48;
constexpr std::size_t kLegacyTorsoImuPayloadSize = 88;
constexpr std::size_t kLegacyFullPayloadSize = 128;
constexpr std::size_t kLegacyBaseFrameSize = kLegacyBasePayloadSize + 3;
constexpr std::size_t kLegacyTorsoImuFrameSize =
    kLegacyTorsoImuPayloadSize + 3;
constexpr std::size_t kLegacyFullFrameSize = kLegacyFullPayloadSize + 3;
constexpr std::size_t kVendorArmEncoderCount = 8;

// 最大 legacy 帧（帧头 + 128 字节 payload + XOR + 帧尾）。
constexpr std::size_t kFrameSize = kLegacyFullFrameSize;
constexpr std::size_t kPayloadSize = kLegacyFullPayloadSize;
constexpr std::uint8_t kFrameHead = 0xAA;
constexpr std::uint8_t kFrameTail = 0x55;
constexpr std::uint8_t kVendorHandsetButtonMask = 0x1F;
constexpr std::uint8_t kVendorHandsetToggleMask = 0x20;
constexpr std::uint8_t kVendorHandsetExtendedButtonMask = 0x1F;
constexpr double kVendorEncoderToRadianRatio =
    6.283185307179586476925286766559 / 16384.0;

enum class ExoskeletonFrameMode
{
    // 严格使用当前 Qnbot Exo Plus 的 131 字节格式。
    Full,
    // 兼容 SDK 中的 51 字节基础格式。
    Base,
    // 兼容 SDK 中的 91 字节“躯干 IMU”格式。
    TorsoImu,
    // 按厂商 SDK 的 legacy 长度集合自动选择，并记忆最近成功长度。
    Auto
};

// 固定容量但带有效长度的帧容器。
//
// 保留 begin/end/fill/operator[] 等 std::array 风格接口，便于离线测试和
// 现有调用方继续构造完整 131 字节样本；auto 模式下 end()/size() 则只
// 暴露实际帧长度。
struct ExoskeletonFrame
{
    std::array<std::uint8_t, kFrameSize> bytes{};
    std::size_t frame_size{kFrameSize};

    std::uint8_t &operator[](const std::size_t index) noexcept
    {
        return bytes[index];
    }

    const std::uint8_t &operator[](const std::size_t index) const noexcept
    {
        return bytes[index];
    }

    void fill(const std::uint8_t value) noexcept
    {
        bytes.fill(value);
    }

    std::uint8_t *data() noexcept { return bytes.data(); }
    const std::uint8_t *data() const noexcept { return bytes.data(); }

    auto begin() noexcept { return bytes.begin(); }
    auto begin() const noexcept { return bytes.begin(); }

    auto end() noexcept { return bytes.begin() + frame_size; }
    auto end() const noexcept { return bytes.begin() + frame_size; }

    std::size_t size() const noexcept { return frame_size; }

    friend bool operator==(
        const ExoskeletonFrame &left,
        const ExoskeletonFrame &right) noexcept
    {
        if (left.frame_size != right.frame_size)
        {
            return false;
        }
        for (std::size_t index = 0; index < left.frame_size; ++index)
        {
            if (left.bytes[index] != right.bytes[index])
            {
                return false;
            }
        }
        return true;
    }

    friend bool operator!=(
        const ExoskeletonFrame &left,
        const ExoskeletonFrame &right) noexcept
    {
        return !(left == right);
    }
};

struct JoystickState
{
    std::int16_t raw_x{0};
    std::int16_t raw_y{0};
    std::int16_t trigger_raw{0};

    // 与厂商 TelemetrySnapshot.joystick_*[2] 一致的完整 16 位 key_mask。
    std::uint16_t key_mask_raw{0};

    // SDK 定义 bit0..4 为 active-low 按键，bit5 为 ON/OFF 拨动开关，
    // bit8..12 为 Plus/RF 扩展键。
    bool toggleOn() const noexcept
    {
        return (key_mask_raw & kVendorHandsetToggleMask) != 0;
    }

    bool buttonsReleased() const noexcept
    {
        return (key_mask_raw & kVendorHandsetButtonMask) ==
               kVendorHandsetButtonMask;
    }

    bool extendedButtonsReleased() const noexcept
    {
        constexpr auto mask = static_cast<std::uint16_t>(
            kVendorHandsetExtendedButtonMask) << 8U;
        return (key_mask_raw & mask) == mask;
    }

    // SDK 定义按键低电平有效；mask 通常使用低 5 位中的单 bit 掩码。
    bool buttonPressed(const std::uint8_t mask) const noexcept
    {
        return mask != 0 && (key_mask_raw & mask) == 0;
    }

    bool extendedButtonPressed(const std::uint8_t mask) const noexcept
    {
        return mask != 0 &&
               (key_mask_raw & (static_cast<std::uint16_t>(mask) << 8U)) == 0;
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
    // 与厂商 TelemetrySnapshot 的 8 个 arm_joint_* 原始/弧度数组对齐。
    // SDK 只定义 8 个槽位，不定义它们对应的机器人关节语义；目标机器人
    // 是否使用某个槽位由独立的现场标定映射层决定。
    std::array<std::int16_t, kVendorArmEncoderCount> left_arm_joint_raw{};
    std::array<std::int16_t, kVendorArmEncoderCount> right_arm_joint_raw{};
    std::array<double, kVendorArmEncoderCount> left_arm_joint_rad{};
    std::array<double, kVendorArmEncoderCount> right_arm_joint_rad{};

    JoystickState left;
    JoystickState right;

    ImuState torso_imu;
    ImuState head_imu;

    // 与 Qnbot SDK 的 format_version 对齐：1=基础数据，2=躯干 IMU，
    // 3=躯干 + 附加 IMU。frame_size 是原始 legacy 完整帧长度。
    std::uint8_t format_version{0};
    std::size_t frame_size{0};

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

bool isSupportedFrameSize(std::size_t frame_size) noexcept;

// 便于不需要保存无状态 parser 对象的调用方使用。
std::uint8_t calculateChecksum(const ExoskeletonFrame &frame) noexcept;
bool isValidFrame(const ExoskeletonFrame &frame) noexcept;
ExoskeletonState parseFrame(const ExoskeletonFrame &frame);

} // namespace robot::input::exoskeleton
