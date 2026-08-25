#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace robot::ti5
{

struct LogicalCanBus
{
    std::string name;
    std::string protocol;
    bool required{false};
    std::vector<std::uint16_t> expected_node_ids;
};

struct DiscoveryOptions
{
    // 保留为运行时配置校验字段；接口枚举由 CanInterfaceManager 完成。
    std::string interface_regex;
    bool require_interface_up{true};

    std::chrono::milliseconds response_timeout{50};
    std::size_t confirmations_required{3};
    std::size_t max_attempts{5};

    bool allow_partial_bus{false};
    bool require_unique_bus_match{true};
};

// 当前 T170C 本体电机统一使用弧度作为运行时单位。
enum class JointUnit
{
    Radian
};

// T170C 本体的编码器运行时配置。
//
// type 和 position_reference 保留为字符串，是为了让配置快照可以直接反映
// YAML 中的声明；Config Loader 会在进入运行时前严格限制为 dual/output。
struct EncoderConfig
{
    std::string type{"dual"};
    std::string position_reference{"output"};
    std::uint32_t counts_per_output_revolution{0};
    double gear_ratio{0.0};
};

// 一个物理 T170C 电机节点的配置。
struct CanMotorConfig
{
    std::uint16_t node_id{0};
    JointUnit unit{JointUnit::Radian};
    EncoderConfig encoder;
};

// robot.yaml 中一个 physical joint 的唯一描述。
// shared_axes 只提供业务别名，不会生成第二个 PhysicalJointConfig。
struct PhysicalJointConfig
{
    std::string name;
    std::string physical_name;
    std::string bus;
    CanMotorConfig motor;
};

struct Ti5RobotConfig
{
    std::string vendor;
    std::string model;
    std::size_t body_motor_count{0};
    std::vector<LogicalCanBus> can_buses;
    EncoderConfig encoder_defaults;
    std::vector<PhysicalJointConfig> joints;
};

// 主机软件允许的电机输出角范围。范围来自 safety.yaml，单位为 rad。
struct JointPositionLimits
{
    double minimum_rad{0.0};
    double maximum_rad{0.0};
    bool verified_on_robot{false};
};

// 关节模型坐标和电机输出坐标之间的安装换算：
// motor_rad = joint_rad * direction + offset_rad。
struct JointCoordinateTransform
{
    double direction{1.0};
    double offset_rad{0.0};
};

struct JointSafetyConfig
{
    std::map<std::string, JointPositionLimits> position_limits;
};

struct KinematicsModelConfig
{
    std::map<std::string, JointCoordinateTransform> joints;
};

struct KinematicsConfig
{
    std::map<std::string, KinematicsModelConfig> models;
};

using LogicalCanBusConfig = LogicalCanBus;
using T170cRobotConfig = Ti5RobotConfig;

struct CanDiscoveryConfig
{
    bool enabled{false};
    bool cache_mapping{false};
    std::string strategy;
    std::chrono::milliseconds response_timeout{0};
    std::size_t confirmations_required{0};
    std::size_t max_attempts{0};
    bool allow_partial_bus{false};
    bool require_unique_bus_match{false};
    bool discover_hands{false};
};

struct CanReceiveConfig
{
    bool centralized_receiver{false};
    bool latest_feedback_cache{false};
    std::string timestamp_clock;
    bool use_can_filters{false};
    bool receive_error_frames{false};
};

struct CanControlConfig
{
    std::size_t frequency_hz{0};
    std::chrono::microseconds inter_frame_gap{0};
    std::chrono::microseconds post_batch_feedback_wait{0};
    std::size_t send_failure_threshold{0};
};

struct CanWatchdogConfig
{
    std::size_t stale_feedback_cycles{0};
    bool reject_new_motion_on_stale_feedback{false};
    bool enter_fault_on_stale_feedback{false};
    bool enter_fault_on_bus_off{false};
};

struct ExclusiveControlConfig
{
    bool enabled{false};
    std::string lock_file;
    bool reject_second_controller{false};
};

// 只描述“哪一块物理 USB-CAN 适配器属于本体”，不描述 waist/head/arm 对应哪个 canX。
struct CanAdapterSelectorConfig
{
    // 支持：usb_serial_short、usb_serial、id_path、sysfs_parent、device_path。
    std::string selector;
    std::string value;
    std::size_t expected_channels{0};
};

struct SocketCanConfig
{
    std::uint32_t bitrate{0};
    std::string interface_regex;
    bool require_interface_up{false};
    bool validate_bitrate{false};
    bool manage_linux_link{false};
    std::chrono::milliseconds restart_ms{0};
    std::chrono::milliseconds reconfigure_wait{100};
    std::chrono::milliseconds startup_wait{100};
    CanAdapterSelectorConfig body_adapter;
};

struct CanConfig
{
    SocketCanConfig socketcan;
    CanDiscoveryConfig discovery;
    CanReceiveConfig receive;
    CanControlConfig control;
    CanWatchdogConfig watchdog;
    ExclusiveControlConfig exclusive_control;
};

} // namespace robot::ti5
