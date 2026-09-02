#include "input/exoskeleton/exoskeleton.hpp"

#include <array>
#include <chrono>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

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

class PseudoTerminal final
{
public:
    PseudoTerminal()
    {
        master_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (master_ < 0)
        {
            throw std::runtime_error(
                std::string{"posix_openpt failed: "} + std::strerror(errno));
        }
        if (::grantpt(master_) != 0 || ::unlockpt(master_) != 0)
        {
            const auto error_number = errno;
            ::close(master_);
            master_ = -1;
            throw std::runtime_error(
                std::string{"initializing pseudo-terminal failed: "} +
                std::strerror(error_number));
        }

        std::array<char, 128> path{};
        const auto error_number = ::ptsname_r(
            master_,
            path.data(),
            path.size());
        if (error_number != 0)
        {
            ::close(master_);
            master_ = -1;
            throw std::runtime_error(
                std::string{"ptsname_r failed: "} +
                std::strerror(error_number));
        }
        slave_ = path.data();
    }

    ~PseudoTerminal()
    {
        if (master_ >= 0)
        {
            ::close(master_);
        }
    }

    PseudoTerminal(const PseudoTerminal &) = delete;
    PseudoTerminal &operator=(const PseudoTerminal &) = delete;

    int master() const noexcept
    {
        return master_;
    }

    const std::string &slave() const noexcept
    {
        return slave_;
    }

    void closeMaster() noexcept
    {
        if (master_ >= 0)
        {
            ::close(master_);
            master_ = -1;
        }
    }

private:
    int master_{-1};
    std::string slave_;
};

void writeInt16(
    ExoskeletonFrame &frame,
    const std::size_t payload_offset,
    const std::int16_t value)
{
    const auto raw = static_cast<std::uint16_t>(value);
    frame[payload_offset + 1] = static_cast<std::uint8_t>(raw & 0xFFU);
    frame[payload_offset + 2] = static_cast<std::uint8_t>(raw >> 8U);
}

ExoskeletonFrame makeFrame()
{
    ExoskeletonFrame frame{};
    frame.frame_size = kLegacyFullFrameSize;
    frame.fill(0);
    frame[0] = kFrameHead;
    frame[frame.size() - 1] = kFrameTail;
    writeInt16(frame, 16, 1234);
    writeInt16(frame, 32, -2345);
    frame[frame.size() - 2] = calculateChecksum(frame);
    return frame;
}

void writeAll(const int descriptor, const std::vector<std::uint8_t> &bytes)
{
    std::size_t offset = 0;
    while (offset < bytes.size())
    {
        const auto result = ::write(
            descriptor,
            bytes.data() + offset,
            bytes.size() - offset);
        if (result > 0)
        {
            offset += static_cast<std::size_t>(result);
            continue;
        }
        if (result < 0 && errno == EINTR)
        {
            continue;
        }
        throw std::runtime_error(
            std::string{"writing pseudo-terminal failed: "} +
            std::strerror(errno));
    }
}

template <typename Predicate>
bool waitFor(Predicate &&predicate, const std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (predicate())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    return predicate();
}

void testReconnectClearsPreviousSnapshot()
{
    PseudoTerminal terminal;

    ExoskeletonConfig config;
    config.device = terminal.slave();
    config.usb_vid = 0x0483;
    config.usb_pid = 0x5740;
    config.baudrate = 115200;
    config.poll_timeout = std::chrono::milliseconds{5};
    config.stale_timeout = std::chrono::milliseconds{150};
    config.reconnect_interval = std::chrono::milliseconds{20};
    config.frame_mode = ExoskeletonFrameMode::Full;

    Exoskeleton exoskeleton{config};
    exoskeleton.start();
    expect(
        waitFor(
            [&] { return exoskeleton.connected(); },
            std::chrono::milliseconds{500}),
        "exoskeleton did not open the pseudo-terminal");

    const auto frame = makeFrame();
    writeAll(
        terminal.master(),
        std::vector<std::uint8_t>{frame.begin(), frame.end()});
    expect(
        waitFor(
            [&]
            {
                const auto state = exoskeleton.latestState();
                return state &&
                       state->left_arm_joint_raw[0] == 1234 &&
                       state->right_arm_joint_raw[0] == -2345;
            },
            std::chrono::milliseconds{500}),
        "exoskeleton did not receive the pseudo-terminal frame");

    terminal.closeMaster();
    expect(
        waitFor(
            [&]
            {
                return !exoskeleton.latestState().has_value() &&
                       !exoskeleton.stateFresh();
            },
            std::chrono::milliseconds{500}),
        "stale exoskeleton snapshot survived the serial disconnect");

    exoskeleton.stop();
}

} // namespace

int main()
{
    try
    {
        testReconnectClearsPreviousSnapshot();
        std::cout << "Exoskeleton runtime tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Exoskeleton runtime test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
