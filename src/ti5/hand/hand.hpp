#pragma once

#include "ti5/hand/hand_channel.hpp"
#include "ti5/hand/hand_config.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace robot::ti5
{

enum class HandSide
{
    Left,
    Right
};

struct HandState
{
    hand::AoyiPositionValues positions_raw{};
    std::optional<hand::AoyiForceValues> forces_raw;
    std::chrono::steady_clock::time_point timestamp{};
};

// TI5 六通道傲意灵巧手部件。
//
// Hand 只暴露已有协议证据的原始通道状态和 0x50 位置命令。
// 手势、张开/闭合语义和抓取力闭环应建立在本层之上。
class Hand final
{
public:
    using PositionValues = hand::AoyiPositionValues;
    using SpeedValues = hand::AoyiSpeedValues;

    Hand(HandSide side,
         const hand::HandSideConfig &config,
         std::string interface_name,
         std::chrono::milliseconds response_timeout);
    Hand(HandSide side,
         const hand::HandSideConfig &config,
         std::unique_ptr<hand::HandTransport> transport,
         std::chrono::milliseconds response_timeout);

    HandSide side() const noexcept;
    const std::string &name() const noexcept;
    std::uint8_t controllerNodeId() const noexcept;
    bool controlAllowed() const noexcept;

    std::optional<HandState> readState();

    // 只有 protocol_verified 和 control_enabled 同时为 true 才允许发送。
    void commandPositionsRaw(const PositionValues &positions,
                             const SpeedValues &speeds);

private:
    static hand::HandSideConfig validatedConfig(
        HandSide side,
        const hand::HandSideConfig &config);

    HandSide side_;
    hand::HandSideConfig config_;
    hand::HandChannel channel_;
};

} // namespace robot::ti5
