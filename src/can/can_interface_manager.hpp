#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::can
{

// Linux SocketCAN 接口及其物理 USB-CAN 身份。
// 该类型不包含 T170C 或傲意手的协议语义。
struct CanInterfaceInfo
{
    std::string name;
    unsigned int ifindex{0};
    bool up{false};

    std::optional<std::uint32_t> bitrate;
    std::optional<std::uint32_t> restart_ms;

    // /sys/class/net/<name>/device 解析后的 canonical sysfs 路径。
    std::string device_path;

    // USB-CAN 的接口设备名，例如 1-1.2.2:1.0。
    std::string sysfs_parent;
    std::string usb_parent;

    // 优先来自 /run/udev/data/n<ifindex>，缺失时回退到 sysfs serial。
    std::optional<std::string> usb_serial_short;
    std::optional<std::string> usb_serial;
    std::optional<std::string> id_serial;
    std::optional<std::string> id_path;
};

// 适配器级 selector。expected_channels=0 表示调用方不要求通道数校验。
struct CanAdapterSelector
{
    // usb_serial_short、usb_serial、id_path、sysfs_parent、device_path。
    std::string kind;
    std::string value;
    std::size_t expected_channels{0};
};

struct CanInterfaceSettings
{
    std::uint32_t bitrate{1000000};
    std::uint32_t restart_ms{100};
    std::chrono::milliseconds reconfigure_wait{100};
    std::chrono::milliseconds startup_wait{100};
    bool validate_bitrate{true};
};

class CanInterfaceManager final
{
public:
    // 枚举所有符合正则的 SocketCAN 接口，包括 DOWN 的接口。
    std::vector<CanInterfaceInfo> enumerate(
        const std::string &interface_regex) const;

    // 读取一个指定接口的当前状态和物理身份。
    CanInterfaceInfo inspect(const std::string &interface_name) const;

    // 从已枚举的接口中选择同一物理适配器的全部通道。
    // 选择失败或通道数与 expected_channels 不符时抛出异常。
    std::vector<CanInterfaceInfo> selectAdapter(
        const std::vector<CanInterfaceInfo> &interfaces,
        const CanAdapterSelector &selector) const;

    // 通过 rtnetlink 将接口配置为目标 bitrate/restart-ms 并拉起，随后复读验证。
    CanInterfaceInfo prepare(
        const std::string &interface_name,
        const CanInterfaceSettings &settings) const;
};

} // namespace robot::can
