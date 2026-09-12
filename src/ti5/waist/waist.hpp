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

enum class WaistControlState
{
    Unprepared,
    Prepared,
    StartingPositionControl,
    PositionControlActive,
    Stopped,
    Failed
};

struct WaistOptions
{
    std::chrono::milliseconds control_period{10};
    std::chrono::microseconds inter_frame_gap{50};
    std::size_t position_control_start_cycles{30};
    std::size_t maximum_stale_cycles{3};
    std::chrono::milliseconds maximum_feedback_age{30};
    double start_position_tolerance_rad{0.012};
};

struct WaistJointState
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

struct WaistState
{
    std::array<WaistJointState, 5> joints{};
    CanBusHealth bus_health{};
    bool all_positions_available{false};
    bool all_csp_feedback_fresh{false};
};

// Five physical waist/fold joints, owned by this component only.
// Position commands use 0x44 and must establish/verify wire mode 8.
// No STOP/brake or mutable Joint interface is exposed. clearFault is an
// explicit 0x0B recovery operation; it never claims to release or hold load.
// ID 2 is a single Joint: waist_pitch is the fold_p3 semantic alias.
// Single-threaded; callers must provide exclusive control of this CAN group.
// Destruction sends no frames. Hardware power-loss holding is not certified here.
class Waist final
{
public:
    static constexpr std::size_t kJointCount = 5;
    // Waist 组件中的腰部俯仰语义对应实体 node 2 / fold_p3。
    static constexpr std::size_t kWaistPitchIndex = 1;
    using JointValues = std::array<double, kJointCount>;
    using JointPositions = std::array<std::optional<double>, kJointCount>;
    using JointNames = std::array<std::string, kJointCount>;

    Waist(std::unique_ptr<CanBus> bus,
         const std::vector<JointConfig> &available_joint_configs,
         WaistOptions options = {});

    const std::string &logicalBusName() const noexcept;
    const JointNames &jointNames() const noexcept;
    WaistControlState controlState() const noexcept;

    const Joint &joint(std::size_t index) const;

    void prepare();
    WaistState readState();
    JointPositions readPositions();
    void clearFault();
    void validatePositions(const JointValues &positions) const;
    void startPositionControlAtCurrentPosition();
    void commandPositionsCsp(const JointValues &positions);

private:
    JointValues queryCurrentPositions();
    std::array<DriverStatus, kJointCount> queryDriverStatuses();
    void requireUniformStartableModes(
        const std::array<DriverStatus, kJointCount> &statuses) const;
    void sendPositions(const JointValues &positions);
    void requireHealthyBus(const char *operation) const;

    std::string logical_bus_name_{"waist_fold"};
    JointNames joint_names_{
        "waist_yaw",
        "fold_p3",
        "fold_p2",
        "fold_p1",
        "fold_r"};
    WaistOptions options_;
    std::unique_ptr<CanBus> bus_;
    std::array<JointConfig, kJointCount> configs_{};
    std::array<std::unique_ptr<Joint>, kJointCount> joints_{};
    JointValues last_commanded_positions_{};
    WaistControlState control_state_{WaistControlState::Unprepared};
};

} // namespace robot::ti5
