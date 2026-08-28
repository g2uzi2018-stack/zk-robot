#include "input/exoskeleton/exoskeleton_stream_decoder.hpp"

#include <algorithm>
#include <stdexcept>

namespace robot::input::exoskeleton
{

std::vector<ExoskeletonFrame> ExoskeletonStreamDecoder::feed(
    const std::uint8_t *data,
    const std::size_t size)
{
    if (size > 0 && data == nullptr)
    {
        throw std::invalid_argument("Exoskeleton decoder data must not be null");
    }

    if (size > 0)
    {
        buffer_.insert(buffer_.end(), data, data + size);
    }

    std::vector<ExoskeletonFrame> frames;
    while (true)
    {
        const auto head = std::find(
            buffer_.begin(),
            buffer_.end(),
            kFrameHead);
        if (head == buffer_.end())
        {
            statistics_.discarded_bytes += buffer_.size();
            buffer_.clear();
            break;
        }

        if (head != buffer_.begin())
        {
            const auto discarded = static_cast<std::size_t>(
                std::distance(buffer_.begin(), head));
            statistics_.discarded_bytes += discarded;
            buffer_.erase(buffer_.begin(), head);
        }

        if (buffer_.size() < kFrameSize)
        {
            break;
        }

        ExoskeletonFrame candidate{};
        std::copy_n(buffer_.begin(), kFrameSize, candidate.begin());

        const bool tail_valid = candidate[kFrameSize - 1] == kFrameTail;
        const bool checksum_valid =
            ExoskeletonProtocol::calculateChecksum(candidate) ==
            candidate[kPayloadSize + 1];

        if (!tail_valid || !checksum_valid)
        {
            if (!tail_valid)
            {
                ++statistics_.tail_failures;
            }
            if (!checksum_valid)
            {
                ++statistics_.checksum_failures;
            }

            // 只丢弃当前候选的第一个字节。剩余字节中可能已经包含
            // 下一帧的帧头，不能整块清空。
            buffer_.erase(buffer_.begin());
            ++statistics_.discarded_bytes;
            continue;
        }

        frames.push_back(candidate);
        ++statistics_.valid_frames;
        buffer_.erase(buffer_.begin(), buffer_.begin() + kFrameSize);
    }

    return frames;
}

void ExoskeletonStreamDecoder::reset() noexcept
{
    buffer_.clear();
    statistics_ = StreamDecoderStatistics{};
}

void ExoskeletonStreamDecoder::resetBuffer() noexcept
{
    buffer_.clear();
}

} // namespace robot::input::exoskeleton
