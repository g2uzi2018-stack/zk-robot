#pragma once

#include "input/exoskeleton/exoskeleton_protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot::input::exoskeleton
{

struct StreamDecoderStatistics
{
    std::uint64_t valid_frames{0};
    std::uint64_t checksum_failures{0};
    std::uint64_t tail_failures{0};
    std::uint64_t discarded_bytes{0};
    std::uint64_t length_switches{0};
};

class ExoskeletonStreamDecoder final
{
public:
    explicit ExoskeletonStreamDecoder(
        ExoskeletonFrameMode mode = ExoskeletonFrameMode::Full);

    std::vector<ExoskeletonFrame> feed(
        const std::uint8_t *data,
        std::size_t size);

    std::vector<ExoskeletonFrame> feed(
        const std::vector<std::uint8_t> &data)
    {
        return feed(data.data(), data.size());
    }

    // 清空缓存和累计统计，适合一次新的解码会话。
    void reset() noexcept;
    // 只清空半帧缓存，保留累计统计，适合串口断线重连。
    void resetBuffer() noexcept;
    std::size_t bufferedBytes() const noexcept { return buffer_.size(); }
    StreamDecoderStatistics statistics() const noexcept { return statistics_; }

private:
    ExoskeletonFrameMode mode_;
    std::vector<std::uint8_t> buffer_;
    std::vector<std::size_t> frame_sizes_;
    std::size_t preferred_frame_size_{0};
    std::size_t preferred_failures_{0};
    StreamDecoderStatistics statistics_;
};

} // namespace robot::input::exoskeleton
