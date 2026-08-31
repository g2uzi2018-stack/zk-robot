#include "input/exoskeleton/serial_device_discovery.hpp"

#include <algorithm>
#include <charconv>
#include <cctype>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace
{

using robot::input::exoskeleton::SerialDeviceSelector;

std::optional<std::string> readTextFile(const std::filesystem::path &path)
{
    std::ifstream input(path);
    if (!input)
    {
        return std::nullopt;
    }

    std::string value;
    std::getline(input, value);
    while (!value.empty() &&
           std::isspace(static_cast<unsigned char>(value.back())) != 0)
    {
        value.pop_back();
    }
    std::size_t first = 0;
    while (first < value.size() &&
           std::isspace(static_cast<unsigned char>(value[first])) != 0)
    {
        ++first;
    }
    if (first != 0)
    {
        value.erase(0, first);
    }
    if (value.empty())
    {
        return std::nullopt;
    }
    return value;
}

std::optional<std::uint16_t> parseHexId(const std::string &text)
{
    std::uint32_t value = 0;
    const auto result = std::from_chars(
        text.data(),
        text.data() + text.size(),
        value,
        16);
    if (result.ec != std::errc{} ||
        result.ptr != text.data() + text.size() ||
        value > 0xFFFFU)
    {
        return std::nullopt;
    }
    return static_cast<std::uint16_t>(value);
}

std::optional<std::pair<std::uint16_t, std::uint16_t>> findUsbIdentity(
    const std::filesystem::path &sysfs_tty_path)
{
    std::error_code error;
    auto current = std::filesystem::canonical(sysfs_tty_path, error);
    if (error)
    {
        return std::nullopt;
    }

    while (true)
    {
        const auto vendor_text = readTextFile(current / "idVendor");
        const auto product_text = readTextFile(current / "idProduct");
        if (vendor_text && product_text)
        {
            const auto vendor = parseHexId(*vendor_text);
            const auto product = parseHexId(*product_text);
            if (vendor && product)
            {
                return std::make_pair(*vendor, *product);
            }
        }

        const auto parent = current.parent_path();
        if (parent == current || current == current.root_path())
        {
            break;
        }
        current = parent;
    }

    return std::nullopt;
}

bool isUsbSerialName(const std::string &name)
{
    return name.rfind("ttyACM", 0) == 0 || name.rfind("ttyUSB", 0) == 0;
}

bool matches(
    const SerialDeviceSelector &selector,
    const std::uint16_t vendor,
    const std::uint16_t product)
{
    return vendor == selector.usb_vid &&
           (selector.match_vid_only || product == selector.usb_pid);
}

} // namespace

namespace robot::input::exoskeleton
{

std::vector<SerialDeviceInfo> enumerateSerialDevices(
    const SerialDeviceSelector &selector,
    const std::filesystem::path &sysfs_tty_root,
    const std::filesystem::path &device_root)
{
    std::vector<SerialDeviceInfo> result;
    std::error_code error;
    const std::filesystem::directory_iterator end;
    for (std::filesystem::directory_iterator iterator{
             sysfs_tty_root,
             std::filesystem::directory_options::skip_permission_denied,
             error};
         !error && iterator != end;
         iterator.increment(error))
    {
        const auto name = iterator->path().filename().string();
        if (!isUsbSerialName(name))
        {
            continue;
        }

        const auto identity = findUsbIdentity(iterator->path());
        if (!identity || !matches(selector, identity->first, identity->second))
        {
            continue;
        }

        const auto device_path = device_root / name;
        error.clear();
        if (!std::filesystem::exists(device_path, error) || error)
        {
            continue;
        }

        std::error_code canonical_error;
        const auto sysfs_path = std::filesystem::canonical(iterator->path(), canonical_error);
        if (canonical_error)
        {
            continue;
        }

        result.push_back({
            device_path,
            sysfs_path,
            identity->first,
            identity->second});
    }

    std::sort(result.begin(), result.end(), [](const auto &left, const auto &right) {
        return left.device_path.string() < right.device_path.string();
    });
    return result;
}

} // namespace robot::input::exoskeleton
