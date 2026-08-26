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

enum class GripperSide
{
    Left,
    Right
};

struct GripperState
{
    hand::AoyiPositionValues positions_raw{};
    std::optional<hand::AoyiForceValues> forces_raw;
    std::chrono::steady_clock::time_point timestamp{};
};

// TI5 的末端不是 TIAGo 两指直线夹爪，而是六通道傲意灵巧手。
// 本类保留 Gripper 这一部件层名称，但只暴露已经有协议证据的原始通道状态
// 和 0x50 命令；open/close、米制开口、抓取力闭环属于后续策略层。
class Gripper final
{
public:
    using PositionValues = hand::AoyiPositionValues;
    using SpeedValues = hand::AoyiSpeedValues;

    Gripper(GripperSide side,
            const hand::HandSideConfig &config,
            std::string interface_name,
            std::chrono::milliseconds response_timeout);
    Gripper(GripperSide side,
            const hand::HandSideConfig &config,
            std::unique_ptr<hand::HandTransport> transport,
            std::chrono::milliseconds response_timeout);

    GripperSide side() const noexcept;
    const std::string &name() const noexcept;
    std::uint8_t controllerNodeId() const noexcept;
    bool controlAllowed() const noexcept;

    std::optional<GripperState> readState();

    // 只有 protocol_verified 和 control_enabled 同时为 true 才允许发送。
    // 当前仓库默认 hands.yaml 两项均未开放，所以实机默认只读。
    void commandPositionsRaw(const PositionValues &positions,
                             const SpeedValues &speeds);

private:
    static hand::HandSideConfig validatedConfig(
        GripperSide side,
        const hand::HandSideConfig &config);

    GripperSide side_;
    hand::HandSideConfig config_;
    hand::HandChannel channel_;
};

} // namespace robot::ti5
