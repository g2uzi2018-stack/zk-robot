#include "input/exoskeleton/exoskeleton_stream_decoder.hpp"

#include <algorithm>
#include <iterator>
#include <stdexcept>

namespace
{

enum class CandidateStatus
{
    NeedMore,
    Invalid,
    Valid
};

CandidateStatus tryCandidate(
    const std::vector<std::uint8_t> &buffer,
    const std::size_t frame_size,
    robot::input::exoskeleton::ExoskeletonFrame &candidate,
    robot::input::exoskeleton::StreamDecoderStatistics &statistics)
{
    if (buffer.size() < frame_size)
    {
        return CandidateStatus::NeedMore;
    }

    candidate = robot::input::exoskeleton::ExoskeletonFrame{};
    candidate.frame_size = frame_size;
    std::copy_n(buffer.begin(), frame_size, candidate.begin());

    const bool tail_valid = candidate[frame_size - 1] ==
                            robot::input::exoskeleton::kFrameTail;
    const bool checksum_valid =
        robot::input::exoskeleton::ExoskeletonProtocol::calculateChecksum(
            candidate) == candidate[frame_size - 2];

    if (!tail_valid || !checksum_valid)
    {
        if (!tail_valid)
        {
            ++statistics.tail_failures;
        }
        if (!checksum_valid)
        {
            ++statistics.checksum_failures;
        }
        return CandidateStatus::Invalid;
    }

    return CandidateStatus::Valid;
}

} // namespace

namespace robot::input::exoskeleton
{

ExoskeletonStreamDecoder::ExoskeletonStreamDecoder(
    const ExoskeletonFrameMode mode)
    : mode_(mode)
{
    switch (mode_)
    {
    case ExoskeletonFrameMode::Full:
        frame_sizes_ = {kLegacyFullFrameSize};
        break;
    case ExoskeletonFrameMode::Base:
        frame_sizes_ = {kLegacyBaseFrameSize};
        break;
    case ExoskeletonFrameMode::TorsoImu:
        frame_sizes_ = {kLegacyTorsoImuFrameSize};
        break;
    case ExoskeletonFrameMode::Auto:
        // 与 QnbotStreamParser 的 legacy_frame_lengths 一致：优先尝试
        // 最大格式，再回退到 91/51 字节格式。
        frame_sizes_ = {
            kLegacyFullFrameSize,
            kLegacyTorsoImuFrameSize,
            kLegacyBaseFrameSize};
        break;
    default:
        throw std::invalid_argument("Unknown exoskeleton frame mode");
    }
}

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

        // frame_sizes_ is non-empty and sorted from longest to shortest.
        const auto minimum_frame_size = frame_sizes_.back();
        if (buffer_.size() < minimum_frame_size)
        {
            break;
        }

        std::vector<std::size_t> attempts;
        attempts.reserve(frame_sizes_.size());
        if (mode_ == ExoskeletonFrameMode::Auto &&
            preferred_frame_size_ != 0)
        {
            attempts.push_back(preferred_frame_size_);
        }
        for (const auto frame_size : frame_sizes_)
        {
            if (std::find(attempts.begin(), attempts.end(), frame_size) ==
                attempts.end())
            {
                attempts.push_back(frame_size);
            }
        }

        bool emitted = false;
        bool needs_more_data = false;
        ExoskeletonFrame decoded{};
        for (const auto frame_size : attempts)
        {
            ExoskeletonFrame candidate{};
            const auto status = tryCandidate(
                buffer_,
                frame_size,
                candidate,
                statistics_);
            if (status == CandidateStatus::NeedMore)
            {
                needs_more_data = true;
                continue;
            }
            if (status == CandidateStatus::Valid)
            {
                decoded = candidate;
                emitted = true;
                if (mode_ == ExoskeletonFrameMode::Auto)
                {
                    if (preferred_frame_size_ != 0 &&
                        preferred_frame_size_ != frame_size)
                    {
                        ++statistics_.length_switches;
                    }
                    preferred_frame_size_ = frame_size;
                    preferred_failures_ = 0;
                }
                break;
            }
        }

        if (emitted)
        {
            frames.push_back(decoded);
            ++statistics_.valid_frames;
            buffer_.erase(
                buffer_.begin(),
                buffer_.begin() + decoded.frame_size);
            continue;
        }

        // 如果较短候选还没有收齐，先等待后续字节。这样在 51/91 字节
        // 格式被分片时不会因为先探测 131 字节而错误丢弃半帧。
        if (needs_more_data)
        {
            break;
        }

        // 只丢当前候选的一个帧头；payload 中出现 0xAA 时仍可继续找
        // 下一处同步点，符合 SDK 的流式重同步语义，同时保留比整块
        // 清空更安全的恢复行为。
        buffer_.erase(buffer_.begin());
        ++statistics_.discarded_bytes;
        if (mode_ == ExoskeletonFrameMode::Auto &&
            preferred_frame_size_ != 0)
        {
            ++preferred_failures_;
            if (preferred_failures_ >= 3)
            {
                preferred_frame_size_ = 0;
                preferred_failures_ = 0;
            }
        }
    }

    return frames;
}

void ExoskeletonStreamDecoder::reset() noexcept
{
    buffer_.clear();
    preferred_frame_size_ = 0;
    preferred_failures_ = 0;
    statistics_ = StreamDecoderStatistics{};
}

void ExoskeletonStreamDecoder::resetBuffer() noexcept
{
    buffer_.clear();
    preferred_frame_size_ = 0;
    preferred_failures_ = 0;
}

} // namespace robot::input::exoskeleton
