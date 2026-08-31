#include "input/exoskeleton/serial_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>
#include <utility>

namespace
{

std::string errorMessage(
    const std::string &operation,
    const int error_number)
{
    return std::string(operation) + ": " + std::strerror(error_number);
}

bool baudrateToTermios(const std::uint32_t baudrate, speed_t &speed)
{
    switch (baudrate)
    {
    case 9600:
        speed = B9600;
        return true;
    case 19200:
        speed = B19200;
        return true;
    case 38400:
        speed = B38400;
        return true;
    case 57600:
        speed = B57600;
        return true;
    case 115200:
        speed = B115200;
        return true;
#ifdef B230400
    case 230400:
        speed = B230400;
        return true;
#endif
#ifdef B460800
    case 460800:
        speed = B460800;
        return true;
#endif
#ifdef B500000
    case 500000:
        speed = B500000;
        return true;
#endif
#ifdef B576000
    case 576000:
        speed = B576000;
        return true;
#endif
#ifdef B921600
    case 921600:
        speed = B921600;
        return true;
#endif
#ifdef B1000000
    case 1000000:
        speed = B1000000;
        return true;
#endif
#ifdef B2000000
    case 2000000:
        speed = B2000000;
        return true;
#endif
#ifdef B3000000
    case 3000000:
        speed = B3000000;
        return true;
#endif
#ifdef B4000000
    case 4000000:
        speed = B4000000;
        return true;
#endif
    default:
        return false;
    }
}

int timeoutMilliseconds(const std::chrono::milliseconds timeout)
{
    if (timeout.count() <= 0)
    {
        return 0;
    }

    if (timeout.count() > INT_MAX)
    {
        return INT_MAX;
    }
    return static_cast<int>(timeout.count());
}

} // namespace

namespace robot::input::exoskeleton
{

SerialTransport::SerialTransport(const std::uint32_t baudrate)
    : baudrate_(baudrate)
{
    if (baudrate_ == 0)
    {
        throw std::invalid_argument("Exoskeleton baudrate must be positive");
    }
}

SerialTransport::SerialTransport(
    std::string device,
    const std::uint32_t baudrate)
    : device_(std::move(device)), baudrate_(baudrate)
{
    if (device_.empty())
    {
        throw std::invalid_argument("Exoskeleton serial device must not be empty");
    }
    if (baudrate_ == 0)
    {
        throw std::invalid_argument("Exoskeleton baudrate must be positive");
    }
}

SerialTransport::~SerialTransport()
{
    close();
}

bool SerialTransport::open()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return openUnlocked();
}

bool SerialTransport::open(const std::string &device)
{
    std::lock_guard<std::mutex> lock(mutex_);
    device_ = device;
    return openUnlocked();
}

bool SerialTransport::openUnlocked()
{
    closeUnlocked();
    last_error_.clear();

    if (device_.empty())
    {
        last_error_ = "Exoskeleton serial device must not be empty";
        return false;
    }

    speed_t speed{};
    if (!baudrateToTermios(baudrate_, speed))
    {
        last_error_ = "Unsupported serial baudrate: " + std::to_string(baudrate_);
        return false;
    }

    int flags = O_RDWR | O_NOCTTY | O_NONBLOCK;
#ifdef O_CLOEXEC
    flags |= O_CLOEXEC;
#endif
    const int candidate = ::open(device_.c_str(), flags);
    if (candidate < 0)
    {
        const int error_number = errno;
        last_error_ = errorMessage("open " + device_, error_number);
        return false;
    }

    termios options{};
    if (::tcgetattr(candidate, &options) != 0)
    {
        const int error_number = errno;
        ::close(candidate);
        last_error_ = errorMessage("tcgetattr " + device_, error_number);
        return false;
    }

    ::cfmakeraw(&options);
    options.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB | CSTOPB));
#ifdef CRTSCTS
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    options.c_cflag |= static_cast<tcflag_t>(CS8 | CLOCAL | CREAD);
    options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (::cfsetispeed(&options, speed) != 0 ||
        ::cfsetospeed(&options, speed) != 0)
    {
        const int error_number = errno;
        ::close(candidate);
        last_error_ = errorMessage("cfset speed " + device_, error_number);
        return false;
    }

    if (::tcsetattr(candidate, TCSANOW, &options) != 0)
    {
        const int error_number = errno;
        ::close(candidate);
        last_error_ = errorMessage("tcsetattr " + device_, error_number);
        return false;
    }

    // 只清空主机端已有接收数据，不向设备发送任何协议字节。
    if (::tcflush(candidate, TCIFLUSH) != 0)
    {
        const int error_number = errno;
        ::close(candidate);
        last_error_ = errorMessage("tcflush " + device_, error_number);
        return false;
    }

    file_descriptor_ = candidate;
    return true;
}

void SerialTransport::closeUnlocked() noexcept
{
    if (file_descriptor_ >= 0)
    {
        ::close(file_descriptor_);
        file_descriptor_ = -1;
    }
}

void SerialTransport::close() noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    closeUnlocked();
}

bool SerialTransport::isOpen() const noexcept
{
    std::lock_guard<std::mutex> lock(mutex_);
    return file_descriptor_ >= 0;
}

std::string SerialTransport::device() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return device_;
}

PollResult SerialTransport::poll(const std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (file_descriptor_ < 0)
    {
        return {TransportStatus::Closed, EBADF};
    }

    pollfd descriptor{};
    descriptor.fd = file_descriptor_;
    descriptor.events = POLLIN | POLLPRI;

    int timeout_ms = timeoutMilliseconds(timeout);
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds{timeout_ms};
    while (true)
    {
        const int result = ::poll(&descriptor, 1, timeout_ms);
        if (result >= 0)
        {
            if (result == 0)
            {
                return {TransportStatus::Timeout, 0};
            }

            if ((descriptor.revents & (POLLIN | POLLPRI)) != 0)
            {
                // POLLIN|POLLHUP 仍可能带有最后一批可读字节，交给 read()
                // 先取走；下一次 read() 会报告断开。
                return {TransportStatus::Ready, 0};
            }
            if ((descriptor.revents & POLLNVAL) != 0)
            {
                return {TransportStatus::Error, EBADF};
            }
            if ((descriptor.revents & POLLHUP) != 0)
            {
                return {TransportStatus::Closed, 0};
            }
            if ((descriptor.revents & POLLERR) != 0)
            {
                return {TransportStatus::Error, EIO};
            }

            return {TransportStatus::Error, EIO};
        }

        if (errno != EINTR)
        {
            const int error_number = errno;
            last_error_ = errorMessage("poll " + device_, error_number);
            return {TransportStatus::Error, error_number};
        }

        // 被信号打断时按原始 deadline 重新计算剩余时间，避免在连续 EINTR
        // 下把一次有限等待意外延长为永久阻塞。
        if (timeout_ms == 0)
        {
            return {TransportStatus::Timeout, 0};
        }

        const auto remaining = std::chrono::duration_cast<
            std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0)
        {
            return {TransportStatus::Timeout, 0};
        }
        timeout_ms = std::max(1, timeoutMilliseconds(remaining));
    }
}

ReadResult SerialTransport::read(
    std::uint8_t *destination,
    const std::size_t capacity)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (file_descriptor_ < 0)
    {
        return {TransportStatus::Closed, 0, EBADF};
    }
    if (capacity == 0)
    {
        return {TransportStatus::Ready, 0, 0};
    }
    if (destination == nullptr)
    {
        return {TransportStatus::Error, 0, EINVAL};
    }

    const ssize_t result = ::read(file_descriptor_, destination, capacity);
    if (result > 0)
    {
        return {
            TransportStatus::Ready,
            static_cast<std::size_t>(result),
            0};
    }
    if (result == 0)
    {
        return {TransportStatus::Closed, 0, 0};
    }

    const int error_number = errno;
    if (error_number == EAGAIN || error_number == EWOULDBLOCK ||
        error_number == EINTR)
    {
        return {TransportStatus::Timeout, 0, 0};
    }

    last_error_ = errorMessage("read " + device_, error_number);
    return {TransportStatus::Error, 0, error_number};
}

std::string SerialTransport::lastError() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
}

} // namespace robot::input::exoskeleton
