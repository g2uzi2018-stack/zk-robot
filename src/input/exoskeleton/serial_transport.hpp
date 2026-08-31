#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace robot::input::exoskeleton
{

enum class TransportStatus
{
    Ready,
    Timeout,
    Closed,
    Error
};

struct PollResult
{
    TransportStatus status{TransportStatus::Closed};
    int error_number{0};
};

struct ReadResult
{
    TransportStatus status{TransportStatus::Closed};
    std::size_t bytes_read{0};
    int error_number{0};
};

// 只负责一个 POSIX 串口的生命周期和字节读取。
// 协议、帧同步和设备状态机均由上层负责。
class SerialTransport final
{
public:
    explicit SerialTransport(std::uint32_t baudrate);
    SerialTransport(std::string device, std::uint32_t baudrate);
    ~SerialTransport();

    SerialTransport(const SerialTransport &) = delete;
    SerialTransport &operator=(const SerialTransport &) = delete;

    bool open();
    bool open(const std::string &device);
    void close() noexcept;

    bool isOpen() const noexcept;

    PollResult poll(std::chrono::milliseconds timeout);
    PollResult poll(int timeout_ms)
    {
        return poll(std::chrono::milliseconds{timeout_ms});
    }

    ReadResult read(std::uint8_t *destination, std::size_t capacity);
    ReadResult read(std::vector<std::uint8_t> &destination)
    {
        return read(destination.data(), destination.size());
    }

    std::string lastError() const;
    std::string device() const;
    std::uint32_t baudrate() const noexcept { return baudrate_; }

private:
    bool openUnlocked();
    void closeUnlocked() noexcept;

    std::string device_;
    const std::uint32_t baudrate_;

    mutable std::mutex mutex_;
    int file_descriptor_{-1};
    std::string last_error_;
};

} // namespace robot::input::exoskeleton
