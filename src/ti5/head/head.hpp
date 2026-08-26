#pragma once

#include "ti5/can/can_bus.hpp"
#include "ti5/joint/joint.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace robot::ti5
{

enum class HeadControlState
{
    Unprepared,
    Prepared,
    StartingPositionControl,
    PositionControlActive,
    RequestingStop,
    Stopped,
    Failed
};

struct HeadOptions
{
    std::chrono::milliseconds control_period{10};
    std::chrono::microseconds inter_frame_gap{50};
    std::size_t position_control_start_cycles{30};
    std::size_t maximum_stale_cycles{3};
    std::chrono::milliseconds maximum_feedback_age{30};
    double start_position_tolerance_rad{0.012};
    std::chrono::milliseconds stop_inter_joint_gap{20};
    std::chrono::milliseconds stop_settle_time{250};
};

struct HeadJointState
{
    std::optional<double> position_rad;
    std::optional<double> velocity_rad_s;
    std::optional<double> current_amps;
    std::optional<std::uint32_t> run_mode;
    std::optional<std::uint32_t> fault_bits;
    std::uint64_t csp_update_sequence{0};
    std::optional<std::chrono::steady_clock::time_point>
        last_csp_feedback_timestamp;
};

struct HeadState
{
    std::array<HeadJointState, 3> joints{};
    CanBusHealth bus_health{};
    bool all_positions_available{false};
    bool all_csp_feedback_fresh{false};
};

// TI5 三自由度头部，固定顺序为 neck_yaw、neck_pitch、neck_roll。
//
// 与 TIAGo Head 的职责一致：管理一条头部总线和一组 Joint，提供部件级
// 状态与完整批次命令。协议能力仍遵循 TI5：不提供虚构的 enable、
// disable、clearFault 或速度参数接口。
class Head final
{
public:
    static constexpr std::size_t kJointCount = 3;
    using JointValues = std::array<double, kJointCount>;
    using JointNames = std::array<std::string, kJointCount>;

    Head(std::unique_ptr<CanBus> bus,
         const std::vector<JointConfig> &available_joint_configs,
         HeadOptions options = {});

    const std::string &logicalBusName() const noexcept;
    const JointNames &jointNames() const noexcept;
    HeadControlState controlState() const noexcept;

    Joint &joint(std::size_t index);
    const Joint &joint(std::size_t index) const;

    void prepare();
    HeadState readState();
    void validatePositions(const JointValues &positions) const;
    void startPositionControlAtCurrentPosition();
    void commandPositionsCsp(const JointValues &positions);
    void requestStopModeAndConfirm();

private:
    JointValues queryCurrentPositions();
    std::array<DriverStatus, kJointCount> queryDriverStatuses();
    void requireUniformStartableModes(
        const std::array<DriverStatus, kJointCount> &statuses) const;
    void sendPositions(const JointValues &positions);
    void requireHealthyBus(const char *operation) const;

    std::string logical_bus_name_{"head"};
    JointNames joint_names_{
        "neck_yaw",
        "neck_pitch",
        "neck_roll"};
    HeadOptions options_;
    std::unique_ptr<CanBus> bus_;
    std::array<JointConfig, kJointCount> configs_{};
    std::array<std::unique_ptr<Joint>, kJointCount> joints_{};
    JointValues last_commanded_positions_{};
    HeadControlState control_state_{HeadControlState::Unprepared};
};

} // namespace robot::ti5
