#include "input/exoskeleton/serial_device_discovery.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace
{

namespace fs = std::filesystem;
using namespace robot::input::exoskeleton;

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

class FakeSysfs final
{
public:
    FakeSysfs()
    {
        const auto suffix = std::to_string(::getpid()) + "_" +
                            std::to_string(
                                std::chrono::steady_clock::now().time_since_epoch().count());
        root_ = fs::temp_directory_path() / ("zk_robot_exoskeleton_" + suffix);
        sysfs_tty_root_ = root_ / "sys/class/tty";
        device_root_ = root_ / "dev";
        fs::create_directories(sysfs_tty_root());
        fs::create_directories(device_root());
    }

    ~FakeSysfs()
    {
        std::error_code error;
        fs::remove_all(root_, error);
    }

    const fs::path &sysfsRoot() const noexcept
    {
        return sysfs_tty_root_;
    }

    const fs::path &deviceRoot() const noexcept
    {
        return device_root_;
    }

    void addDevice(
        const std::string &tty_name,
        const std::string &usb_parent,
        const std::string &vid,
        const std::string &pid)
    {
        const auto usb_path = root_ / "sys/devices/usb" / usb_parent;
        const auto tty_path = usb_path / (usb_parent + ":1.0") / "tty" / tty_name;
        fs::create_directories(tty_path);
        write(usb_path / "idVendor", vid);
        write(usb_path / "idProduct", pid);
        fs::create_symlink(tty_path, sysfs_tty_root_ / tty_name);
        write(device_root_ / tty_name, "fake device");
    }

private:
    fs::path sysfs_tty_root() const
    {
        return root_ / "sys/class/tty";
    }

    fs::path device_root() const
    {
        return root_ / "dev";
    }

    static void write(const fs::path &path, const std::string &value)
    {
        std::ofstream output(path);
        if (!output)
        {
            throw std::runtime_error("failed to create fake sysfs file: " + path.string());
        }
        output << value << '\n';
    }

    fs::path root_;
    fs::path sysfs_tty_root_{root_ / "sys/class/tty"};
    fs::path device_root_{root_ / "dev"};
};

void testVidPidMatching()
{
    FakeSysfs fake;
    fake.addDevice("ttyACM7", "1-1", "0483", "5740");
    fake.addDevice("ttyUSB2", "2-1", "0483", "5741");
    fake.addDevice("ttyACM8", "3-1", "1234", "5740");
    fake.addDevice("ttyS9", "4-1", "0483", "5740");

    const SerialDeviceSelector strict{0x0483, 0x5740, false};
    const auto exact = enumerateSerialDevices(
        strict,
        fake.sysfsRoot(),
        fake.deviceRoot());
    expect(exact.size() == 1, "strict VID:PID matching selected the wrong ports");
    expect(exact.front().device_path == fake.deviceRoot() / "ttyACM7",
           "matching did not return the runtime serial path");
    expect(exact.front().usb_vid == 0x0483 && exact.front().usb_pid == 0x5740,
           "USB identity was not retained");

    const SerialDeviceSelector vid_only{0x0483, 0xFFFF, true};
    const auto vendor_matches = enumerateSerialDevices(
        vid_only,
        fake.sysfsRoot(),
        fake.deviceRoot());
    expect(vendor_matches.size() == 2,
           "VID-only matching did not include both matching USB serial ports");
    expect(vendor_matches[0].device_path == fake.deviceRoot() / "ttyACM7" &&
               vendor_matches[1].device_path == fake.deviceRoot() / "ttyUSB2",
           "USB serial matches were not sorted deterministically");
}

} // namespace

int main()
{
    try
    {
        testVidPidMatching();
        std::cout << "Exoskeleton device discovery tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Exoskeleton device discovery test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
