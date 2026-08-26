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

enum class ArmSide
{
    Left,
    Right
};

// Arm 只记录硬件部件所处阶段；轨迹执行中的 Running/Completed 等状态
// 由后续 ArmController 负责。
enum class ArmControlState
{
    Unprepared,
    Prepared,
    StartingPositionControl,
    PositionControlActive,
    RequestingStop,
    Stopped,
    Failed
};

struct ArmOptions
{
    std::chrono::milliseconds control_period{10};
    std::chrono::microseconds inter_frame_gap{50};
    std::size_t position_control_start_cycles{30};
    std::size_t maximum_stale_cycles{3};
    std::chrono::milliseconds maximum_feedback_age{30};
    double start_position_tolerance_rad{0.012};

    // 现有双臂 STOP 实机工具使用的保守间隔。STOP 只代表 mode=0，
    // 不代表去使能、释放转矩或抱闸动作。
    std::chrono::milliseconds stop_inter_joint_gap{20};
    std::chrono::milliseconds stop_settle_time{250};
};

struct ArmJointState
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

struct ArmState
{
    std::array<ArmJointState, 7> joints{};
    CanBusHealth bus_health{};
    bool all_positions_available{false};
    bool all_csp_feedback_fresh{false};
};

// 一条 TI5 七自由度机械臂。
//
// Arm 负责：
//   - 按语义名称固定组装左臂或右臂的 7 个 Joint；
//   - 独占该机械臂的一条逻辑 CAN 总线；
//   - 汇总整臂反馈并在发送前检查完整的 7 轴目标；
//   - 使用当前安全位置建立并验证 Position CSP；
//   - 请求 0x02 STOP 并查询确认 7 轴均进入 mode=0。
//
// Arm 不负责：
//   - CAN 接口发现和 YAML 解析；
//   - 轨迹插值、速度/加速度规划和控制线程；
//   - 自然下垂越过驱动器目标范围后的肩部边界恢复；
//   - 抱闸、伺服电源或未经确认的 0x01 enable。
class Arm final
{
public:
    static constexpr std::size_t kJointCount = 7;

    using JointValues = std::array<double, kJointCount>;
    using JointNames = std::array<std::string, kJointCount>;

    // available_joint_configs 可以包含整机全部 JointConfig；Arm 按 name
    // 选择本侧所需的 7 个关节，不依赖输入顺序。
    Arm(ArmSide side,
        std::unique_ptr<CanBus> bus,
        const std::vector<JointConfig> &available_joint_configs,
        ArmOptions options = {});

    ArmSide side() const noexcept;
    const std::string &logicalBusName() const noexcept;
    const JointNames &jointNames() const noexcept;
    ArmControlState controlState() const noexcept;
    // 自最近一次 prepare 起是否至少成功发送过一帧 0x44。
    bool hasSentPositionCommand() const noexcept;

    Joint &joint(std::size_t index);
    const Joint &joint(std::size_t index) const;

    // 只读准备：查询 7 轴位置、驱动器目标范围、mode 和 fault。
    // 不发送 0x44、0x02 或任何参数写入。
    void prepare();

    ArmState readState();

    // 先验证完整 7 轴目标；任意一轴非法时，本批次不发送任何帧。
    void validatePositions(const JointValues &positions) const;

    // TI5 没有已确认的独立 enable 帧。本接口以 7 轴当前安全位置连续
    // 发送 0x44，验证新反馈、位置包络、mode=8 和 fault=0 后才成功。
    void startPositionControlAtCurrentPosition();

    // 只接受后续控制周期已经规划好的位置点，不在 Arm 内生成轨迹。
    void commandPositionsCsp(const JointValues &positions);

    // 从腕部到肩部发送 0x02，并查询确认所有关节 mode=0、fault=0。
    // 这是显式恢复接口，所以 prepare 失败后仍允许调用。
    // 调用方必须在机械上可靠托住手臂，不能假定 STOP 会释放或保持负载。
    void requestStopModeAndConfirm();

private:
    static JointNames expectedJointNames(ArmSide side);
    static std::string expectedBusName(ArmSide side);

    JointValues queryCurrentPositions();
    std::array<DriverStatus, kJointCount> queryDriverStatuses();
    void requireUniformStartableModes(
        const std::array<DriverStatus, kJointCount> &statuses) const;
    void sendPositions(const JointValues &positions);
    void requireHealthyBus(const char *operation) const;

    ArmSide side_;
    std::string logical_bus_name_;
    JointNames joint_names_;
    ArmOptions options_;

    // CanBus 必须先于所有 Joint 构造并晚于所有 Joint 析构。
    std::unique_ptr<CanBus> bus_;
    std::array<JointConfig, kJointCount> configs_{};
    std::array<std::unique_ptr<Joint>, kJointCount> joints_{};

    JointValues last_commanded_positions_{};
    bool position_command_sent_{false};
    ArmControlState control_state_{ArmControlState::Unprepared};
};

} // namespace robot::ti5
