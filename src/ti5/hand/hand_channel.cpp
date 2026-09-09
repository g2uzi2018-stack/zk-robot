#include "ti5/hand/hand_channel.hpp"

#include "can/socket_can.hpp"

#include <stdexcept>
#include <utility>

namespace robot::ti5::hand
{
namespace
{

class SocketCanHandTransport final : public HandTransport
{
public:
    SocketCanHandTransport(std::string interface_name,
                           const std::uint8_t hand_id)
        : socket_(
              std::move(interface_name),
              robot::can::SocketCanOptions{{hand_id}, true})
    {
    }

    void send(const robot::can::CanFrame &frame) override
    {
        socket_.send(frame);
    }

    std::optional<robot::can::CanFrame> receive(
        const std::chrono::milliseconds timeout) override
    {
        return socket_.receive(timeout);
    }

private:
    robot::can::SocketCan socket_;
};

void validateConstruction(const std::unique_ptr<HandTransport> &transport,
                          const std::uint8_t controller_id,
                          const std::uint8_t response_id,
                          const std::chrono::milliseconds response_timeout)
{
    if (!transport)
    {
        throw std::invalid_argument(
            "Aoyi HandChannel transport must not be null");
    }
    if (controller_id == 0)
    {
        throw std::invalid_argument(
            "Aoyi HandChannel controller ID must be non-zero");
    }
    if (response_id == 0)
    {
        throw std::invalid_argument(
            "Aoyi HandChannel response ID must be non-zero");
    }
    if (response_timeout.count() <= 0)
    {
        throw std::invalid_argument(
            "Aoyi HandChannel response timeout must be positive");
    }
}

} // namespace

HandChannel::HandChannel(
    std::string interface_name,
    const std::uint8_t controller_id,
    const std::uint8_t response_id,
    const std::chrono::milliseconds response_timeout)
    : HandChannel(
          std::make_unique<SocketCanHandTransport>(
              std::move(interface_name), controller_id),
          controller_id,
          response_id,
          response_timeout)
{
}

HandChannel::HandChannel(
    std::unique_ptr<HandTransport> transport,
    const std::uint8_t controller_id,
    const std::uint8_t response_id,
    const std::chrono::milliseconds response_timeout)
    : transport_(std::move(transport)),
      controller_id_(controller_id),
      response_id_(response_id),
      response_timeout_(response_timeout),
      reassembler_(controller_id, response_id, response_timeout)
{
    validateConstruction(
        transport_, controller_id_, response_id_, response_timeout_);
}

HandChannel::HandChannel(
    std::string interface_name,
    const std::uint8_t hand_id,
    const std::chrono::milliseconds response_timeout)
    : HandChannel(
          std::move(interface_name), hand_id, hand_id, response_timeout)
{
}

HandChannel::HandChannel(
    std::unique_ptr<HandTransport> transport,
    const std::uint8_t hand_id,
    const std::chrono::milliseconds response_timeout)
    : HandChannel(std::move(transport), hand_id, hand_id, response_timeout)
{
}

std::uint8_t HandChannel::handId() const noexcept
{
    return controller_id_;
}

std::uint8_t HandChannel::responseId() const noexcept
{
    return response_id_;
}

void HandChannel::sendPacket(
    const std::uint8_t command,
    const std::vector<std::uint8_t> &payload)
{
    const auto bytes = encodePacket(controller_id_, command, payload);
    for (const auto &frame : fragmentPacket(controller_id_, bytes))
    {
        transport_->send(frame);
    }
}

std::optional<AoyiHandStatus> HandChannel::queryStatus()
{
    // 丢弃调用前残留的旧帧，要求本次查询返回新包。
    while (transport_->receive(std::chrono::milliseconds{0}))
    {
    }
    reassembler_.reset();
    sendPacket(kAoyiGetStatusCommand, {kAoyiGetStatusSubcommand});

    const auto deadline =
        std::chrono::steady_clock::now() + response_timeout_;
    while (true)
    {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline)
        {
            return std::nullopt;
        }
        auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - now);
        if (remaining.count() <= 0)
        {
            remaining = std::chrono::milliseconds{1};
        }
        const auto frame = transport_->receive(remaining);
        if (!frame)
        {
            return std::nullopt;
        }
        const auto packet = reassembler_.push(*frame, now);
        if (!packet || packet->hand_id != response_id_ ||
            packet->command != kAoyiGetStatusCommand)
        {
            continue;
        }
        return decodeStatusPayload(packet->payload);
    }
}

void HandChannel::commandPositions(
    const AoyiPositionValues &positions,
    const AoyiSpeedValues &speeds)
{
    sendPacket(
        kAoyiSetPositionsCommand,
        encodePositionPayload(positions, speeds));
}

} // namespace robot::ti5::hand
