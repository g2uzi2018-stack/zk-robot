#include "input/exoskeleton/exoskeleton_stream_decoder.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
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

template <typename Callable>
void expectThrows(Callable &&callable, const std::string &message)
{
    try
    {
        callable();
    }
    catch (const std::exception &)
    {
        return;
    }
    throw std::runtime_error(message);
}

ExoskeletonFrame makeFrame(const std::uint8_t payload_value)
{
    ExoskeletonFrame frame{};
    frame.fill(payload_value);
    frame[0] = kFrameHead;
    frame[kFrameSize - 1] = kFrameTail;
    frame[kPayloadSize + 1] = calculateChecksum(frame);
    return frame;
}

std::vector<std::uint8_t> bytes(const ExoskeletonFrame &frame)
{
    return {frame.begin(), frame.end()};
}

void append(
    std::vector<ExoskeletonFrame> &destination,
    std::vector<ExoskeletonFrame> source)
{
    destination.insert(
        destination.end(),
        source.begin(),
        source.end());
}

void expectOneFrame(
    const std::vector<ExoskeletonFrame> &frames,
    const ExoskeletonFrame &expected,
    const std::string &message)
{
    expect(frames.size() == 1 && frames.front() == expected, message);
}

void testFragmentationAndCoalescing()
{
    const auto first = makeFrame(0x11);
    const auto second = makeFrame(0x22);

    ExoskeletonStreamDecoder half_decoder;
    const auto first_bytes = bytes(first);
    const auto split = first_bytes.size() / 2;
    expect(half_decoder.feed(first_bytes.data(), split).empty(),
           "half frame should not be emitted");
    expectOneFrame(
        half_decoder.feed(first_bytes.data() + split, first_bytes.size() - split),
        first,
        "half frame was not reassembled");

    ExoskeletonStreamDecoder byte_decoder;
    std::vector<ExoskeletonFrame> byte_frames;
    for (const auto value : first_bytes)
    {
        append(byte_frames, byte_decoder.feed(&value, 1));
    }
    expect(byte_frames.size() == 1 && byte_frames.front() == first,
           "byte-at-a-time feed failed");

    std::vector<std::uint8_t> joined = bytes(first);
    const auto second_bytes = bytes(second);
    joined.insert(joined.end(), second_bytes.begin(), second_bytes.end());
    ExoskeletonStreamDecoder joined_decoder;
    const auto joined_frames = joined_decoder.feed(joined);
    expect(joined_frames.size() == 2 && joined_frames[0] == first &&
               joined_frames[1] == second,
           "two coalesced frames were not emitted separately");

    const auto statistics = joined_decoder.statistics();
    expect(statistics.valid_frames == 2 && joined_decoder.bufferedBytes() == 0,
           "coalesced frame statistics/buffer state failed");
}

void testNoiseAndRandomChunks()
{
    const auto first = makeFrame(0x31);
    const auto second = makeFrame(0x32);
    std::vector<std::uint8_t> stream{0x00, 0x01, 0x7F, 0x10};
    const auto first_bytes = bytes(first);
    const auto second_bytes = bytes(second);
    stream.insert(stream.end(), first_bytes.begin(), first_bytes.end());
    stream.insert(stream.end(), second_bytes.begin(), second_bytes.end());

    ExoskeletonStreamDecoder decoder;
    std::vector<ExoskeletonFrame> frames;
    std::size_t offset = 0;
    const std::array<std::size_t, 8> chunk_sizes{1, 7, 3, 29, 2, 64, 5, 100};
    std::size_t chunk_index = 0;
    while (offset < stream.size())
    {
        const auto size = std::min(
            chunk_sizes[chunk_index++ % chunk_sizes.size()],
            stream.size() - offset);
        append(frames, decoder.feed(stream.data() + offset, size));
        offset += size;
    }

    expect(frames.size() == 2 && frames[0] == first && frames[1] == second,
           "random chunk feed failed");
    expect(decoder.statistics().discarded_bytes == 4,
           "leading noise discard count failed");
}

void testBadCandidatesResynchronize()
{
    const auto good = makeFrame(0x42);

    auto bad_checksum = makeFrame(0x41);
    bad_checksum[kPayloadSize + 1] ^= 0x01;
    std::vector<std::uint8_t> checksum_stream = bytes(bad_checksum);
    const auto good_bytes = bytes(good);
    checksum_stream.insert(
        checksum_stream.end(),
        good_bytes.begin(),
        good_bytes.end());
    ExoskeletonStreamDecoder checksum_decoder;
    const auto checksum_frames = checksum_decoder.feed(checksum_stream);
    expectOneFrame(
        checksum_frames,
        good,
        "decoder did not recover after checksum failure");
    expect(checksum_decoder.statistics().checksum_failures >= 1,
           "checksum failure was not recorded");

    auto bad_tail = makeFrame(0x43);
    bad_tail[kFrameSize - 1] = 0x54;
    std::vector<std::uint8_t> tail_stream = bytes(bad_tail);
    tail_stream.insert(tail_stream.end(), good_bytes.begin(), good_bytes.end());
    ExoskeletonStreamDecoder tail_decoder;
    const auto tail_frames = tail_decoder.feed(tail_stream);
    expectOneFrame(
        tail_frames,
        good,
        "decoder did not recover after tail failure");
    expect(tail_decoder.statistics().tail_failures >= 1,
           "tail failure was not recorded");

    // 丢掉坏候选中的任意一个字节后，后面的帧头仍然必须被找到。
    auto dropped_stream = bytes(makeFrame(0x51));
    dropped_stream.erase(dropped_stream.begin() + 37);
    dropped_stream.insert(
        dropped_stream.end(),
        good_bytes.begin(),
        good_bytes.end());
    ExoskeletonStreamDecoder dropped_decoder;
    const auto dropped_frames = dropped_decoder.feed(dropped_stream);
    expectOneFrame(
        dropped_frames,
        good,
        "decoder did not recover after a dropped byte");
}

void testPayloadHeadAndInputValidation()
{
    const auto payload_head_frame = makeFrame(kFrameHead);
    ExoskeletonStreamDecoder decoder;
    const auto frames = decoder.feed(bytes(payload_head_frame));
    expectOneFrame(
        frames,
        payload_head_frame,
        "0xAA inside payload must not create an early frame");

    expectThrows(
        [&] { decoder.feed(nullptr, 1); },
        "decoder must reject a null non-empty input");
}

} // namespace

int main()
{
    try
    {
        testFragmentationAndCoalescing();
        testNoiseAndRandomChunks();
        testBadCandidatesResynchronize();
        testPayloadHeadAndInputValidation();
        std::cout << "Exoskeleton stream decoder tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Exoskeleton stream decoder test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
