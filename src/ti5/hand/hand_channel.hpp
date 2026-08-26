#pragma once

#include "can/can_frame.hpp"
#include "ti5/hand/aoyi_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace robot::ti5::hand
{

class HandTransport
{
public:
    virtual ~HandTransport() = default;
    virtual void send(const robot::can::CanFrame &frame) = 0;
    virtual std::optional<robot::can::CanFrame> receive(
        std::chrono::milliseconds timeout) = 0;
};

// 一个傲意手控制器通道。负责应用包分片、重组和一次只读状态事务，
// 不包含抓取语义、手势、限位或“是否抓住物体”的判断。
class HandChannel final
{
public:
    HandChannel(std::string interface_name,
                std::uint8_t hand_id,
                std::chrono::milliseconds response_timeout);
    HandChannel(std::unique_ptr<HandTransport> transport,
                std::uint8_t hand_id,
                std::chrono::milliseconds response_timeout);

    std::uint8_t handId() const noexcept;
    std::optional<AoyiHandStatus> queryStatus();
    void commandPositions(const AoyiPositionValues &positions,
                          const AoyiSpeedValues &speeds);

private:
    void sendPacket(std::uint8_t command,
                    const std::vector<std::uint8_t> &payload);

    std::unique_ptr<HandTransport> transport_;
    std::uint8_t hand_id_{0};
    std::chrono::milliseconds response_timeout_{0};
    PacketReassembler reassembler_;
};

} // namespace robot::ti5::hand
