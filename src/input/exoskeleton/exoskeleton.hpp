#pragma once

#include "input/exoskeleton/exoskeleton_protocol.hpp"
#include "input/exoskeleton/exoskeleton_stream_decoder.hpp"
#include "input/exoskeleton/serial_transport.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace robot::input::exoskeleton
{

struct ExoskeletonConfig
{
    // 非空且不是 "auto" 时直接使用该串口路径；"auto" 或空字符串时，
    // 通过 USB VID:PID 枚举唯一设备。显式路径与官方读取器的
    // port 参数对应。实际运行配置推荐使用 "auto"，避免依赖会变化的
    // /dev/ttyACM* 或 /dev/ttyUSB* 名称。
    std::string device;
    std::uint16_t usb_vid{0x0483};
    std::uint16_t usb_pid{0x5740};
    bool match_vid_only{false};
    std::uint32_t baudrate{2000000};
    std::chrono::milliseconds poll_timeout{20};
    std::chrono::milliseconds stale_timeout{100};
    std::chrono::milliseconds reconnect_interval{1000};
    ExoskeletonFrameMode frame_mode{ExoskeletonFrameMode::Full};
};

ExoskeletonConfig loadExoskeletonConfig(
    const std::filesystem::path &config_path);

struct ExoskeletonStatistics
{
    std::uint64_t received_bytes{0};
    std::uint64_t valid_frames{0};
    std::uint64_t checksum_failures{0};
    std::uint64_t tail_failures{0};
    std::uint64_t discarded_bytes{0};
    std::uint64_t length_switches{0};
    // 包含首次成功打开串口；每次后续成功重连再增加一次。
    std::uint64_t reconnect_count{0};
};

class Exoskeleton final
{
public:
    explicit Exoskeleton(ExoskeletonConfig config);
    ~Exoskeleton();

    Exoskeleton(const Exoskeleton &) = delete;
    Exoskeleton &operator=(const Exoskeleton &) = delete;

    void start();
    void stop() noexcept;

    bool connected() const noexcept;
    std::string device() const;
    std::optional<ExoskeletonState> latestState() const;
    bool stateFresh() const;
    ExoskeletonStatistics statistics() const;

private:
    using Clock = std::chrono::steady_clock;

    void run();
    void waitForReconnect();
    void consumeBytes(const std::uint8_t *data, std::size_t size);
    void clearLatestState();

    ExoskeletonConfig config_;
    SerialTransport transport_;
    ExoskeletonStreamDecoder decoder_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex lifecycle_mutex_;
    std::thread reader_thread_;

    mutable std::mutex state_mutex_;
    std::optional<ExoskeletonState> latest_state_;

    mutable std::mutex statistics_mutex_;
    ExoskeletonStatistics statistics_;

    std::mutex wait_mutex_;
    std::condition_variable wait_condition_;
};

} // namespace robot::input::exoskeleton
