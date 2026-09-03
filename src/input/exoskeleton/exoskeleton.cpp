#include "input/exoskeleton/exoskeleton.hpp"

#include "common/logger.hpp"
#include "input/exoskeleton/serial_device_discovery.hpp"

#include <charconv>
#include <limits>
#include <stdexcept>
#include <utility>

#include <yaml-cpp/yaml.h>

namespace
{

[[noreturn]] void throwConfigError(
    const std::string &context,
    const std::string &message)
{
    throw std::invalid_argument(context + ": " + message);
}

YAML::Node loadYamlFile(const std::filesystem::path &path)
{
    try
    {
        return YAML::LoadFile(path.string());
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            path.string(),
            "unable to read or parse YAML: " + std::string(error.what()));
    }
}

YAML::Node requireMap(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsMap())
    {
        throwConfigError(
            context + "." + key,
            "must be a YAML mapping");
    }
    return value;
}

YAML::Node requireScalar(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsScalar())
    {
        throwConfigError(
            context + "." + key,
            "must be a YAML scalar");
    }
    return value;
}

YAML::Node optionalMap(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value)
    {
        return {};
    }
    if (!value.IsMap())
    {
        throwConfigError(
            context + "." + key,
            "must be a YAML mapping when present");
    }
    return value;
}

bool hasKey(const YAML::Node &parent, const char *key)
{
    return static_cast<bool>(parent[key]);
}

std::string requireString(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    try
    {
        return requireScalar(parent, key, context).as<std::string>();
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "must be a string: " + std::string(error.what()));
    }
}

std::string optionalString(
    const YAML::Node &parent,
    const char *key,
    const std::string &default_value,
    const std::string &context)
{
    if (!hasKey(parent, key))
    {
        return default_value;
    }
    return requireString(parent, key, context);
}

std::uint64_t requireUnsigned(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    try
    {
        const auto value = requireScalar(parent, key, context).as<std::int64_t>();
        if (value < 0)
        {
            throwConfigError(
                context + "." + key,
                "must not be negative");
        }
        return static_cast<std::uint64_t>(value);
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "must be a non-negative integer: " + std::string(error.what()));
    }
}

bool requireBool(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    try
    {
        return requireScalar(parent, key, context).as<bool>();
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "must be a boolean: " + std::string(error.what()));
    }
}

std::uint16_t requireUsbId(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto text = requireString(parent, key, context);
    const bool hexadecimal =
        text.size() > 2 && text[0] == '0' &&
        (text[1] == 'x' || text[1] == 'X');
    const auto digits = text.substr(hexadecimal ? 2 : 0);
    std::uint32_t value = 0;
    const auto parsed = std::from_chars(
        digits.data(),
        digits.data() + digits.size(),
        value,
        hexadecimal ? 16 : 10);
    if (digits.empty() || parsed.ec != std::errc{} ||
        parsed.ptr != digits.data() + digits.size() ||
        value == 0 || value > std::numeric_limits<std::uint16_t>::max())
    {
        throwConfigError(
            context + "." + key,
            "must be a positive 16-bit USB id, written as decimal or 0x-prefixed hexadecimal");
    }
    return static_cast<std::uint16_t>(value);
}

std::chrono::milliseconds requireMilliseconds(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = requireUnsigned(parent, key, context);
    using Rep = std::chrono::milliseconds::rep;
    if (value == 0 || value > static_cast<std::uint64_t>(
                              std::numeric_limits<Rep>::max()))
    {
        throwConfigError(
            context + "." + key,
            "must be greater than zero and fit in milliseconds");
    }
    return std::chrono::milliseconds{static_cast<Rep>(value)};
}

std::chrono::milliseconds optionalMilliseconds(
    const YAML::Node &parent,
    const char *key,
    const std::chrono::milliseconds default_value,
    const std::string &context)
{
    if (!hasKey(parent, key))
    {
        return default_value;
    }
    return requireMilliseconds(parent, key, context);
}

robot::input::exoskeleton::ExoskeletonFrameMode parseFrameMode(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    if (!hasKey(parent, key))
    {
        return robot::input::exoskeleton::ExoskeletonFrameMode::Full;
    }

    const auto text = requireString(parent, key, context);
    if (text == "auto" || text == "AUTO" || text == "Auto")
    {
        return robot::input::exoskeleton::ExoskeletonFrameMode::Auto;
    }

    std::uint32_t frame_size = 0;
    const auto parsed = std::from_chars(
        text.data(),
        text.data() + text.size(),
        frame_size,
        10);
    if (parsed.ec != std::errc{} ||
        parsed.ptr != text.data() + text.size())
    {
        throwConfigError(
            context + "." + key,
            "must be auto, 51, 91, or 131");
    }

    switch (frame_size)
    {
    case robot::input::exoskeleton::kLegacyBaseFrameSize:
        return robot::input::exoskeleton::ExoskeletonFrameMode::Base;
    case robot::input::exoskeleton::kLegacyTorsoImuFrameSize:
        return robot::input::exoskeleton::ExoskeletonFrameMode::TorsoImu;
    case robot::input::exoskeleton::kLegacyFullFrameSize:
        return robot::input::exoskeleton::ExoskeletonFrameMode::Full;
    default:
        throwConfigError(
            context + "." + key,
            "must be auto, 51, 91, or 131");
    }
}

const char *transportStatusName(
    const robot::input::exoskeleton::TransportStatus status) noexcept
{
    using robot::input::exoskeleton::TransportStatus;
    switch (status)
    {
    case TransportStatus::Ready:
        return "ready";
    case TransportStatus::Timeout:
        return "timeout";
    case TransportStatus::Closed:
        return "closed";
    case TransportStatus::Error:
        return "error";
    }
    return "unknown";
}

} // namespace

namespace robot::input::exoskeleton
{

ExoskeletonConfig loadExoskeletonConfig(
    const std::filesystem::path &config_path)
{
    const auto root = loadYamlFile(config_path);
    const auto context = config_path.string();
    if (!root || !root.IsMap())
    {
        throwConfigError(context, "root must be a YAML mapping");
    }

    const auto exoskeleton = requireMap(root, "exoskeleton", context);
    const auto exoskeleton_context = context + ".exoskeleton";

    YAML::Node serial = optionalMap(
        exoskeleton,
        "serial",
        exoskeleton_context);
    if (!serial)
    {
        serial = exoskeleton;
    }
    const auto serial_context = exoskeleton_context + ".serial";

    YAML::Node telemetry = optionalMap(
        exoskeleton,
        "telemetry",
        exoskeleton_context);
    if (!telemetry)
    {
        telemetry = exoskeleton;
    }
    const auto telemetry_context = exoskeleton_context + ".telemetry";

    ExoskeletonConfig result;
    result.device = optionalString(serial, "device", "", serial_context);
    result.usb_vid = requireUsbId(serial, "usb_vid", serial_context);
    result.usb_pid = requireUsbId(serial, "usb_pid", serial_context);
    result.match_vid_only = requireBool(
        serial,
        "match_vid_only",
        serial_context);

    const auto baudrate = requireUnsigned(
        serial,
        "baudrate",
        serial_context);
    if (baudrate == 0 || baudrate > std::numeric_limits<std::uint32_t>::max())
    {
        throwConfigError(
            serial_context + ".baudrate",
            "must fit in a positive uint32_t");
    }
    result.baudrate = static_cast<std::uint32_t>(baudrate);
    result.poll_timeout = optionalMilliseconds(
        serial,
        "poll_timeout_ms",
        std::chrono::milliseconds{20},
        serial_context);
    result.reconnect_interval = optionalMilliseconds(
        serial,
        "reconnect_interval_ms",
        std::chrono::milliseconds{1000},
        serial_context);
    result.stale_timeout = optionalMilliseconds(
        telemetry,
        "stale_timeout_ms",
        std::chrono::milliseconds{100},
        telemetry_context);
    result.frame_mode = parseFrameMode(
        telemetry,
        "frame_size",
        telemetry_context);
    return result;
}

Exoskeleton::Exoskeleton(ExoskeletonConfig config)
    : config_(std::move(config)),
      transport_(config_.baudrate),
      decoder_(config_.frame_mode)
{
    if (config_.usb_vid == 0)
    {
        throw std::invalid_argument("Exoskeleton USB VID must be positive");
    }
    if (config_.usb_pid == 0)
    {
        throw std::invalid_argument("Exoskeleton USB PID must be positive");
    }
    if (config_.baudrate == 0)
    {
        throw std::invalid_argument("Exoskeleton baudrate must be positive");
    }
    if (config_.poll_timeout <= std::chrono::milliseconds::zero())
    {
        throw std::invalid_argument("Exoskeleton poll timeout must be positive");
    }
    if (config_.stale_timeout <= std::chrono::milliseconds::zero())
    {
        throw std::invalid_argument("Exoskeleton stale timeout must be positive");
    }
    if (config_.reconnect_interval <= std::chrono::milliseconds::zero())
    {
        throw std::invalid_argument(
            "Exoskeleton reconnect interval must be positive");
    }
}

Exoskeleton::~Exoskeleton()
{
    stop();
}

void Exoskeleton::start()
{
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (running_.load())
    {
        return;
    }

    decoder_.reset();
    connected_.store(false);
    clearLatestState();
    {
        std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
        statistics_ = ExoskeletonStatistics{};
    }

    running_.store(true);
    reader_thread_ = std::thread(&Exoskeleton::run, this);
}

void Exoskeleton::stop() noexcept
{
    std::thread thread;
    {
        std::lock_guard<std::mutex> lock(lifecycle_mutex_);
        if (!running_.load() && !reader_thread_.joinable())
        {
            connected_.store(false);
            transport_.close();
            clearLatestState();
            return;
        }
        running_.store(false);
        thread = std::move(reader_thread_);
    }

    connected_.store(false);
    // poll/read 都受 SerialTransport 内部 mutex 保护；close 会在当前
    // 一次有限 poll 返回后取得该锁，从而唤醒退出路径。
    transport_.close();
    wait_condition_.notify_all();
    if (thread.joinable())
    {
        thread.join();
    }
    clearLatestState();
}

bool Exoskeleton::connected() const noexcept
{
    return connected_.load();
}

std::string Exoskeleton::device() const
{
    return transport_.device();
}

std::optional<ExoskeletonState> Exoskeleton::latestState() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}

void Exoskeleton::clearLatestState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_.reset();
}

bool Exoskeleton::stateFresh() const
{
    if (!connected_.load())
    {
        return false;
    }

    const auto state = latestState();
    if (!state || state->timestamp == Clock::time_point{})
    {
        return false;
    }

    const auto age = Clock::now() - state->timestamp;
    return age >= Clock::duration::zero() && age <= config_.stale_timeout;
}

ExoskeletonStatistics Exoskeleton::statistics() const
{
    std::lock_guard<std::mutex> lock(statistics_mutex_);
    return statistics_;
}

void Exoskeleton::waitForReconnect()
{
    std::unique_lock<std::mutex> lock(wait_mutex_);
    wait_condition_.wait_for(
        lock,
        config_.reconnect_interval,
        [this] { return !running_.load(); });
}

void Exoskeleton::consumeBytes(
    const std::uint8_t *data,
    const std::size_t size)
{
    const auto frames = decoder_.feed(data, size);
    const auto decoder_statistics = decoder_.statistics();
    {
        std::lock_guard<std::mutex> lock(statistics_mutex_);
        statistics_.received_bytes += size;
        statistics_.valid_frames = decoder_statistics.valid_frames;
        statistics_.checksum_failures = decoder_statistics.checksum_failures;
        statistics_.tail_failures = decoder_statistics.tail_failures;
        statistics_.discarded_bytes = decoder_statistics.discarded_bytes;
        statistics_.length_switches = decoder_statistics.length_switches;
    }

    for (const auto &frame : frames)
    {
        try
        {
            auto state = ExoskeletonProtocol::parse(frame);
            state.timestamp = Clock::now();
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = std::move(state);
        }
        catch (const std::exception &error)
        {
            // StreamDecoder 已经验证过帧格式；这里仍保留异常边界，
            // 确保异常数据不会终止串口读取线程。
            robot::common::logger()->error(
                "Exoskeleton: failed to parse a valid frame: {}",
                error.what());
        }
    }
}

void Exoskeleton::run()
{
    const auto logger = robot::common::logger();
    std::array<std::uint8_t, 4096> read_buffer{};
    const SerialDeviceSelector selector{
        config_.usb_vid,
        config_.usb_pid,
        config_.match_vid_only};

    while (running_.load())
    {
        if (!transport_.isOpen())
        {
            connected_.store(false);
            // 旧串口会话的快照不能跨连接继续使用；否则在较短的
            // reconnect_interval 配置下，stateFresh() 可能暂时返回 true。
            clearLatestState();
            decoder_.resetBuffer();
            std::string device;
            if (!config_.device.empty() && config_.device != "auto")
            {
                device = config_.device;
            }
            else
            {
                const auto devices = enumerateSerialDevices(selector);
                if (devices.size() != 1)
                {
                    if (devices.empty())
                    {
                        logger->warn(
                            "Exoskeleton: no serial device matches VID:PID "
                            "0x{:04x}:0x{:04x}; retrying in {} ms",
                            static_cast<unsigned int>(config_.usb_vid),
                            static_cast<unsigned int>(config_.usb_pid),
                            config_.reconnect_interval.count());
                    }
                    else
                    {
                        logger->error(
                            "Exoskeleton: found {} serial devices matching "
                            "VID:PID 0x{:04x}:0x{:04x}; refusing ambiguous "
                            "selection",
                            devices.size(),
                            static_cast<unsigned int>(config_.usb_vid),
                            static_cast<unsigned int>(config_.usb_pid));
                    }
                    waitForReconnect();
                    continue;
                }
                device = devices.front().device_path.string();
            }

            if (!transport_.open(device))
            {
                logger->error(
                    "Exoskeleton: failed to open serial device {}: {}; "
                    "retrying in {} ms",
                    device,
                    transport_.lastError(),
                    config_.reconnect_interval.count());
                waitForReconnect();
                continue;
            }

            decoder_.resetBuffer();
            clearLatestState();
            connected_.store(true);
            logger->info(
                "Exoskeleton connected: device={}, baudrate={}",
                device,
                config_.baudrate);
            {
                std::lock_guard<std::mutex> lock(statistics_mutex_);
                ++statistics_.reconnect_count;
            }
        }

        const auto poll_result = transport_.poll(config_.poll_timeout);
        if (!running_.load())
        {
            break;
        }
        if (poll_result.status == TransportStatus::Timeout)
        {
            continue;
        }
        if (poll_result.status != TransportStatus::Ready)
        {
            const auto detail = transport_.lastError();
            if (poll_result.status == TransportStatus::Closed)
            {
                logger->warn(
                    "Exoskeleton serial connection closed on {} "
                    "(status={}, errno={}); reconnecting",
                    transport_.device(),
                    transportStatusName(poll_result.status),
                    poll_result.error_number);
            }
            else
            {
                logger->error(
                    "Exoskeleton serial poll failed on {} "
                    "(status={}, errno={}): {}; reconnecting",
                    transport_.device(),
                    transportStatusName(poll_result.status),
                    poll_result.error_number,
                    detail.empty() ? "unknown error" : detail);
            }
            connected_.store(false);
            clearLatestState();
            transport_.close();
            decoder_.resetBuffer();
            waitForReconnect();
            continue;
        }

        const auto read_result = transport_.read(
            read_buffer.data(),
            read_buffer.size());
        if (read_result.status == TransportStatus::Ready &&
            read_result.bytes_read > 0)
        {
            consumeBytes(read_buffer.data(), read_result.bytes_read);
            continue;
        }
        if (read_result.status == TransportStatus::Timeout)
        {
            continue;
        }

        const auto detail = transport_.lastError();
        if (read_result.status == TransportStatus::Closed)
        {
            logger->warn(
                "Exoskeleton serial connection closed while reading {} "
                "(status={}, errno={}); reconnecting",
                transport_.device(),
                transportStatusName(read_result.status),
                read_result.error_number);
        }
        else
        {
            logger->error(
                "Exoskeleton serial read failed on {} "
                "(status={}, errno={}): {}; reconnecting",
                transport_.device(),
                transportStatusName(read_result.status),
                read_result.error_number,
                detail.empty() ? "no bytes read" : detail);
        }

        connected_.store(false);
        clearLatestState();
        transport_.close();
        decoder_.resetBuffer();
        waitForReconnect();
    }

    transport_.close();
    connected_.store(false);
    clearLatestState();
}

} // namespace robot::input::exoskeleton
