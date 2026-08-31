#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

namespace robot::input::exoskeleton
{

struct SerialDeviceSelector
{
    std::uint16_t usb_vid{0};
    std::uint16_t usb_pid{0};
    bool match_vid_only{false};
};

struct SerialDeviceInfo
{
    std::filesystem::path device_path;
    std::filesystem::path sysfs_path;
    std::uint16_t usb_vid{0};
    std::uint16_t usb_pid{0};
};

// Enumerate USB-backed serial ports whose USB parent matches the selector.
// The roots are parameters so the matching logic can be tested without
// touching the host's real device tree.
std::vector<SerialDeviceInfo> enumerateSerialDevices(
    const SerialDeviceSelector &selector,
    const std::filesystem::path &sysfs_tty_root = "/sys/class/tty",
    const std::filesystem::path &device_root = "/dev");

} // namespace robot::input::exoskeleton
