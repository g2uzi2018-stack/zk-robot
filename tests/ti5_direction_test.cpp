#include "can/socket_can.hpp"
#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"

#include "ti5/can/can_bus.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/can/encoder_conversion.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/motor/can_motor.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace
{

using namespace std::chrono_literals;

constexpr auto kControlPeriod = 10ms;
constexpr int kReadyProbeCycles = 30;
constexpr int kHoldCycles = 30;
constexpr int kMaximumStaleCycles = 3;
constexpr double kDefaultDeltaRad = 0.001;
constexpr double kMinimumDeltaRad = 0.0001;
constexpr double kMaximumDeltaRad = 0.005;
constexpr double kLimitMarginRad = 0.02;
constexpr double kAllowedOvershootRad = 0.02;
constexpr int kDriverBoundaryCaptureCycles = 250;
constexpr double kDriverBoundaryCaptureOvershootRad = 0.01;
constexpr double kMaximumDriverBoundaryCaptureRad = 0.12;
constexpr double kDriverBoundaryCaptureMarginRad = 0.005;
constexpr double kMaximumShoulderRollRecoveryTravelRad = 1.62;
constexpr double kMaximumDriverBoundaryCaptureSpeedRadPerSecond = 0.20;
constexpr auto kKeyReleaseTimeout = 150ms;

constexpr std::array<const char *, 3> kDirectionBusOrder{
    "head",
    "left_arm",
    "right_arm",
};

constexpr std::array<const char *, 17> kDirectionOrder{
    "neck_yaw",
    "neck_pitch",
    "neck_roll",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow_yaw",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "left_wrist_roll",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow_yaw",
    "right_wrist_pitch",
    "right_wrist_yaw",
    "right_wrist_roll",
};

std::atomic<bool> stop_requested{false};

void signalHandler(int)
{
    stop_requested.store(true);
}

class SingleProcessLock final
{
public:
    SingleProcessLock()
    {
        constexpr const char *lock_path =
            "/tmp/zk_robot_ti5_motion.lock";

        fd_ = ::open(lock_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0 && errno == ENOENT)
        {
            fd_ = ::open(lock_path,
                         O_CREAT | O_EXCL | O_RDONLY |
                             O_CLOEXEC | O_NOFOLLOW,
                         0666);
            if (fd_ < 0 && errno == EEXIST)
            {
                fd_ = ::open(lock_path,
                             O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
            }
        }
        if (fd_ < 0)
        {
            throw std::runtime_error(
                "无法打开 TI5 实机运动互斥锁");
        }
        static_cast<void>(::fchmod(fd_, 0666));
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
        {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "另一个 zk_robot 实机运动程序正在运行");
        }
    }

    ~SingleProcessLock()
    {
        if (fd_ >= 0)
        {
            static_cast<void>(::flock(fd_, LOCK_UN));
            ::close(fd_);
        }
    }

    SingleProcessLock(const SingleProcessLock &) = delete;
    SingleProcessLock &operator=(const SingleProcessLock &) = delete;

private:
    int fd_{-1};
};

bool processNamedIsRunning(const std::string &expected_name)
{
    const auto self = static_cast<long>(::getpid());
    std::error_code error;
    const std::filesystem::directory_iterator end;
    for (std::filesystem::directory_iterator entry{
             "/proc",
             std::filesystem::directory_options::skip_permission_denied,
             error};
         !error && entry != end;
         entry.increment(error))
    {
        const auto pid_text = entry->path().filename().string();
        if (pid_text.empty() ||
            !std::all_of(pid_text.begin(),
                         pid_text.end(),
                         [](const unsigned char value)
                         { return value >= '0' && value <= '9'; }))
        {
            continue;
        }

        long pid = 0;
        try
        {
            pid = std::stol(pid_text);
        }
        catch (const std::exception &)
        {
            continue;
        }
        if (pid == self)
        {
            continue;
        }

        std::ifstream comm{entry->path() / "comm"};
        std::string process_name;
        if (std::getline(comm, process_name) &&
            process_name == expected_name)
        {
            return true;
        }
    }
    return false;
}

void requireExclusiveController()
{
    constexpr std::array<const char *, 4> controller_names{
        "Ti5Control",
        "joint_manager",
        "ti5_follow_arm",
        "robot",
    };
    for (const char *name : controller_names)
    {
        if (processNamedIsRunning(name))
        {
            throw std::runtime_error(
                std::string{"检测到其他控制程序："} + name +
                "；方向测试要求独占本体 CAN");
        }
    }
}

void requireExplicitConfirmation()
{
    const char *value =
        std::getenv("ZK_ROBOT_CONFIRM_DIRECTION_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "未设置 ZK_ROBOT_CONFIRM_DIRECTION_TEST=YES；未打开 CAN");
    }
}

struct Options
{
    double delta_rad{kDefaultDeltaRad};
};

void printUsage(const char *program)
{
    std::cout
        << "用法：\n"
        << "  " << program << " [--delta-rad 数值]\n\n"
        << "默认每个 10 ms 控制周期移动 ±0.001 rad。\n"
        << "测试顺序：头部 Yaw/Pitch/Roll，再从左臂到右臂，各臂由肩到腕。\n"
        << "--delta-rad     设置按住方向键时每周期的增量，范围 0.0001～0.005 rad。\n";
}

Options parseOptions(const int argc, char **argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument{argv[index]};
        if (argument == "--help" || argument == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (argument != "--delta-rad" && argument != "--step-rad")
        {
            throw std::invalid_argument("未知参数：" + argument);
        }
        if (index + 1 >= argc)
        {
            throw std::invalid_argument("--delta-rad 缺少数值");
        }
        try
        {
            options.delta_rad = std::stod(argv[++index]);
        }
        catch (const std::exception &)
        {
            throw std::invalid_argument("--delta-rad 不是有效数值");
        }
    }

    if (!std::isfinite(options.delta_rad) ||
        options.delta_rad < kMinimumDeltaRad ||
        options.delta_rad > kMaximumDeltaRad)
    {
        throw std::invalid_argument(
            "--delta-rad 必须在 0.0001～0.005 rad 之间");
    }
    return options;
}

std::filesystem::path configPath(const char *name)
{
#ifdef TI5_SOURCE_DIR
    const std::filesystem::path root{TI5_SOURCE_DIR};
#else
    const std::filesystem::path root{"."};
#endif
    return root / "config" / "ti5" / "t170c" / name;
}

struct SafetyLimit
{
    double minimum{0.0};
    double maximum{0.0};
};

std::map<std::string, SafetyLimit> loadSafetyLimits(
    const std::filesystem::path &path)
{
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node limits = root["position_limits"];
    if (!limits || !limits.IsMap())
    {
        throw std::runtime_error(
            "safety.yaml 缺少 position_limits");
    }

    std::map<std::string, SafetyLimit> result;
    for (const auto &entry : limits)
    {
        const auto name = entry.first.as<std::string>();
        const auto value = entry.second;
        const SafetyLimit limit{
            value["min"].as<double>(),
            value["max"].as<double>(),
        };
        if (!std::isfinite(limit.minimum) ||
            !std::isfinite(limit.maximum) ||
            limit.minimum >= limit.maximum)
        {
            throw std::runtime_error("无效的软件限位：" + name);
        }
        result.emplace(name, limit);
    }
    return result;
}

std::vector<robot::ti5::PhysicalJointConfig> orderedDirectionJoints(
    const robot::ti5::Ti5RobotConfig &robot_config)
{
    std::map<std::string, robot::ti5::PhysicalJointConfig> by_name;
    for (const auto &joint : robot_config.joints)
    {
        if (joint.bus == "head" ||
            joint.bus == "left_arm" ||
            joint.bus == "right_arm")
        {
            if (!by_name.emplace(joint.name, joint).second)
            {
                throw std::runtime_error(
                    "robot.yaml 中存在重复关节：" + joint.name);
            }
        }
    }

    std::vector<robot::ti5::PhysicalJointConfig> result;
    result.reserve(kDirectionOrder.size());
    for (const char *name : kDirectionOrder)
    {
        const auto it = by_name.find(name);
        if (it == by_name.end())
        {
            throw std::runtime_error(
                "方向测试缺少关节配置：" + std::string{name});
        }
        result.push_back(it->second);
    }
    return result;
}

std::vector<std::string> prepareBodyCan(
    const robot::ti5::CanConfig &can_config)
{
    robot::can::CanInterfaceManager manager;
    const auto all_interfaces =
        manager.enumerate(can_config.socketcan.interface_regex);
    if (all_interfaces.empty())
    {
        throw std::runtime_error("没有找到 SocketCAN 接口");
    }

    const auto &selector = can_config.socketcan.body_adapter;
    const auto body_interfaces = manager.selectAdapter(
        all_interfaces,
        robot::can::CanAdapterSelector{
            selector.selector,
            selector.value,
            selector.expected_channels});
    if (body_interfaces.empty())
    {
        throw std::runtime_error(
            "没有找到匹配的四通道本体 USB-CAN 适配器");
    }

    const robot::can::CanInterfaceSettings settings{
        can_config.socketcan.bitrate,
        static_cast<std::uint32_t>(
            can_config.socketcan.restart_ms.count()),
        can_config.socketcan.reconfigure_wait,
        can_config.socketcan.startup_wait,
        can_config.socketcan.validate_bitrate};

    std::vector<std::string> result;
    result.reserve(body_interfaces.size());
    for (const auto &interface : body_interfaces)
    {
        robot::common::logger()->info(
            "准备本体 CAN {}",
            interface.name);
        const auto ready = manager.prepare(interface.name, settings);
        if (!ready.up)
        {
            throw std::runtime_error(
                interface.name + " 拉起失败");
        }
        if (can_config.socketcan.validate_bitrate &&
            (!ready.bitrate ||
             *ready.bitrate != can_config.socketcan.bitrate))
        {
            throw std::runtime_error(
                interface.name + " 波特率校验失败");
        }
        robot::common::logger()->info(
            "{} 已就绪：bitrate={}，restart-ms={}",
            ready.name,
            ready.bitrate
                ? std::to_string(*ready.bitrate)
                : "unknown",
            ready.restart_ms
                ? std::to_string(*ready.restart_ms)
                : "unknown");
        result.push_back(ready.name);
    }
    return result;
}

std::map<std::string, std::string> discoverDirectionBuses(
    const robot::ti5::Ti5RobotConfig &robot_config,
    const robot::ti5::CanConfig &can_config,
    const std::vector<std::string> &candidate_interfaces)
{
    std::vector<robot::ti5::LogicalCanBus> direction_buses;
    for (const auto &bus : robot_config.can_buses)
    {
        if (std::any_of(
                kDirectionBusOrder.begin(),
                kDirectionBusOrder.end(),
                [&bus](const char *name)
                { return bus.name == name; }))
        {
            direction_buses.push_back(bus);
        }
    }
    if (direction_buses.size() != kDirectionBusOrder.size())
    {
        throw std::runtime_error(
            "robot.yaml 必须包含 head、left_arm 和 right_arm 三条总线");
    }

    robot::ti5::CanDiscovery discovery;
    const auto discovery_result = discovery.discover(
        direction_buses,
        robot::ti5::makeDiscoveryOptions(can_config),
        candidate_interfaces);
    if (!discovery_result.success)
    {
        throw std::runtime_error("头部与双臂 CAN 总线发现失败");
    }

    std::map<std::string, std::string> mapping;
    for (const char *bus_name : kDirectionBusOrder)
    {
        const auto it = std::find_if(
            discovery_result.logical_buses.begin(),
            discovery_result.logical_buses.end(),
            [bus_name](const auto &bus)
            { return bus.bus_name == bus_name; });
        if (it == discovery_result.logical_buses.end() ||
            !it->complete || !it->interface_name)
        {
            throw std::runtime_error(
                std::string{"头部与双臂逻辑总线不完整："} +
                bus_name);
        }
        mapping.emplace(bus_name, *it->interface_name);
    }
    return mapping;
}

struct JointRuntime
{
    robot::ti5::PhysicalJointConfig config;
    SafetyLimit safety;
    robot::ti5::DriverPositionLimits driver_limits;
    std::unique_ptr<robot::ti5::CanMotor> motor;
    double start_position{0.0};
    double last_measured{0.0};
    std::uint64_t last_sequence{0};
    double lower_limit{0.0};
    double upper_limit{0.0};
    double delta_rad{0.0};
    std::optional<double> driver_boundary_capture_target;
    bool driver_boundary_capture_command_sent{false};
    std::string skip_reason;
};

struct DriverStatus
{
    std::int32_t mode{0};
    std::int32_t fault{0};
};

std::int32_t decodeLittleEndianInt32(
    const std::array<std::uint8_t, 8> &data,
    const std::size_t offset)
{
    const std::uint32_t raw =
        static_cast<std::uint32_t>(data[offset]) |
        (static_cast<std::uint32_t>(data[offset + 1]) << 8U) |
        (static_cast<std::uint32_t>(data[offset + 2]) << 16U) |
        (static_cast<std::uint32_t>(data[offset + 3]) << 24U);
    std::int32_t result{0};
    static_assert(sizeof(result) == sizeof(raw));
    std::memcpy(&result, &raw, sizeof(result));
    return result;
}

std::optional<std::int32_t> queryInt32Status(
    robot::can::SocketCan &socket,
    const std::uint16_t node_id,
    const std::uint8_t command,
    const std::chrono::milliseconds timeout)
{
    while (socket.receive(0ms))
    {
    }

    robot::can::CanFrame query{};
    query.id = node_id;
    query.data_length = 1;
    query.data[0] = command;
    socket.send(query);

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline)
    {
        auto remaining = std::chrono::duration_cast<
            std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now());
        if (remaining.count() <= 0)
        {
            remaining = 1ms;
        }
        const auto frame = socket.receive(remaining);
        if (!frame)
        {
            return std::nullopt;
        }
        if (frame->id == node_id &&
            frame->data_length == 5 &&
            frame->data[0] == command)
        {
            return decodeLittleEndianInt32(frame->data, 1);
        }
    }
    return std::nullopt;
}

DriverStatus queryDriverStatus(
    const JointRuntime &joint,
    const std::map<std::string, std::string> &mapping)
{
    robot::can::SocketCan socket{mapping.at(joint.config.bus)};
    const auto mode = queryInt32Status(
        socket,
        joint.config.motor.node_id,
        0x03,
        80ms);
    const auto fault = queryInt32Status(
        socket,
        joint.config.motor.node_id,
        0x0A,
        80ms);
    if (!mode || !fault)
    {
        throw std::runtime_error(
            joint.config.name + " 的模式或故障查询失败");
    }
    return DriverStatus{*mode, *fault};
}

bool isShoulderRoll(const std::string &joint_name)
{
    return joint_name == "left_shoulder_roll" ||
           joint_name == "right_shoulder_roll";
}

bool isKnownShoulderRecoveryCorridor(
    const robot::ti5::PhysicalJointConfig &joint,
    const robot::ti5::DriverPositionLimits &driver_limits,
    const double position)
{
    if (!isShoulderRoll(joint.name))
    {
        return false;
    }
    if (joint.name == "left_shoulder_roll")
    {
        return position < driver_limits.minimum_rad &&
               position >= -kMaximumShoulderRollRecoveryTravelRad;
    }
    return position > driver_limits.maximum_rad &&
           position <= kMaximumShoulderRollRecoveryTravelRad;
}

std::vector<JointRuntime> buildRuntimes(
    const std::vector<robot::ti5::PhysicalJointConfig> &joint_configs,
    const std::map<std::string, SafetyLimit> &safety_limits,
    std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> &buses,
    const double delta_rad)
{
    std::vector<JointRuntime> result;
    result.reserve(joint_configs.size());
    for (const auto &config : joint_configs)
    {
        const auto safety = safety_limits.find(config.name);
        if (safety == safety_limits.end())
        {
            throw std::runtime_error(
                "safety.yaml 缺少软件限位：" + config.name);
        }
        const auto bus = buses.find(config.bus);
        if (bus == buses.end() || !bus->second)
        {
            throw std::runtime_error(
                "没有为关节创建 CAN 总线：" + config.bus);
        }

        auto motor = std::make_unique<robot::ti5::CanMotor>(
            config.motor, *bus->second);
        const auto queried_position = motor->queryPosition();
        const auto csp = motor->queryCspStatus();
        if (!queried_position || !csp)
        {
            throw std::runtime_error(
                "关节初始位置查询失败：" + config.name);
        }

        const double q0 = robot::ti5::positionCountsToRadians(
            csp->position_counts,
            config.motor.encoder.counts_per_output_revolution);
        if (std::abs(q0 - *queried_position) > 0.01)
        {
            throw std::runtime_error(
                "0x08 与 0x41 位置不一致：" + config.name);
        }

        const auto driver_limits = motor->queryPositionLimits();
        if (!driver_limits)
        {
            throw std::runtime_error(
                "驱动器位置目标范围查询失败：" + config.name);
        }

        const auto state = motor->latestState();
        if (!state || !state->position_counts)
        {
            throw std::runtime_error(
                "没有初始 CSP 反馈：" + config.name);
        }

        JointRuntime runtime;
        runtime.config = config;
        runtime.safety = safety->second;
        runtime.driver_limits = *driver_limits;
        runtime.motor = std::move(motor);
        runtime.start_position = q0;
        runtime.last_measured = q0;
        runtime.last_sequence = state->update_sequence;
        runtime.lower_limit = std::max(
            runtime.safety.minimum,
            runtime.driver_limits.minimum_rad);
        runtime.upper_limit = std::min(
            runtime.safety.maximum,
            runtime.driver_limits.maximum_rad);
        runtime.delta_rad = delta_rad;

        const bool outside_driver_limit =
            q0 < runtime.driver_limits.minimum_rad ||
            q0 > runtime.driver_limits.maximum_rad;
        bool known_shoulder_recovery = false;
        if (isKnownShoulderRecoveryCorridor(
                config,
                runtime.driver_limits,
                q0))
        {
            const double capture_lower =
                runtime.driver_limits.minimum_rad +
                kDriverBoundaryCaptureMarginRad;
            const double capture_upper =
                runtime.driver_limits.maximum_rad -
                kDriverBoundaryCaptureMarginRad;
            if (capture_lower >= capture_upper)
            {
                runtime.skip_reason =
                    "驱动器目标范围过窄，无法建立边界接管目标";
            }
            else
            {
                const double capture_target = std::clamp(
                    q0, capture_lower, capture_upper);
                const double capture_distance =
                    std::abs(capture_target - q0);
                if (capture_target < runtime.safety.minimum ||
                    capture_target > runtime.safety.maximum)
                {
                    runtime.skip_reason =
                        "边界接管目标超出 safety.yaml 软件限位";
                }
                else if (capture_distance >
                         kMaximumDriverBoundaryCaptureRad)
                {
                    std::ostringstream reason;
                    reason << std::fixed << std::setprecision(6)
                           << "边界内目标=" << capture_target
                           << " rad，实际接管距离=" << capture_distance
                           << " rad，超过上限 "
                           << kMaximumDriverBoundaryCaptureRad << " rad";
                    runtime.skip_reason = reason.str();
                }
                else if (capture_distance > 1e-6)
                {
                    runtime.driver_boundary_capture_target =
                        capture_target;
                    known_shoulder_recovery = true;
                }
            }
        }

        if (runtime.skip_reason.empty() &&
            runtime.lower_limit >= runtime.upper_limit)
        {
            runtime.skip_reason = "软件限位与驱动器限位没有交集";
        }
        else if (runtime.skip_reason.empty() && outside_driver_limit &&
                 !known_shoulder_recovery)
        {
            std::ostringstream reason;
            reason << std::fixed << std::setprecision(6)
                   << "当前位置=" << q0
                   << " rad 超出驱动器目标范围";
            if (isShoulderRoll(config.name))
            {
                reason << "，且不在正确侧的肩横滚被动下垂包络 ±"
                       << kMaximumShoulderRollRecoveryTravelRad
                       << " rad 内";
            }
            runtime.skip_reason = reason.str();
        }
        else if (runtime.skip_reason.empty() &&
                 !known_shoulder_recovery &&
                 (q0 < runtime.lower_limit - kAllowedOvershootRad ||
                 q0 > runtime.upper_limit + kAllowedOvershootRad)
        )
        {
            runtime.skip_reason =
                "当前位置已经超出有效限位范围";
        }
        result.push_back(std::move(runtime));
    }
    return result;
}

void captureAtDriverBoundary(
    JointRuntime &joint,
    const std::map<std::string, std::string> &mapping)
{
    if (!joint.driver_boundary_capture_target)
    {
        return;
    }

    const double original_position = joint.start_position;
    const double capture_target =
        *joint.driver_boundary_capture_target;
    const double planned_distance =
        std::abs(capture_target - original_position);
    if (planned_distance > kMaximumDriverBoundaryCaptureRad)
    {
        throw std::runtime_error(
            joint.config.name + " 的驱动器边界接管距离超过上限");
    }

    const auto initial_status = queryDriverStatus(joint, mapping);
    if (initial_status.fault != 0 ||
        (initial_status.mode != 0 && initial_status.mode != 8))
    {
        throw std::runtime_error(
            joint.config.name +
            " 当前 mode/fault 不允许进行驱动器边界接管");
    }

    robot::common::logger()->warn(
        "{} 开始驱动器边界接管：{:.6f} -> {:.6f} rad，计划位移 {:.6f} rad",
        joint.config.name,
        original_position,
        capture_target,
        planned_distance);

    double previous_measured = original_position;
    double maximum_observed_speed = 0.0;
    int stale_cycles = 0;
    auto previous_feedback_time = std::chrono::steady_clock::now();
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 0;
         cycle < kDriverBoundaryCaptureCycles;
         ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error(
                "操作者在驱动器边界接管阶段中止");
        }

        next_cycle += kControlPeriod;
        joint.motor->commandPositionCsp(capture_target);
        joint.driver_boundary_capture_command_sent = true;
        std::this_thread::sleep_until(next_cycle);

        const auto state = joint.motor->latestState();
        if (!state || !state->position_counts ||
            state->update_sequence <= joint.last_sequence)
        {
            ++stale_cycles;
            if (stale_cycles >= kMaximumStaleCycles)
            {
                throw std::runtime_error(
                    joint.config.name +
                    " 的驱动器边界接管反馈连续失效");
            }
            continue;
        }

        stale_cycles = 0;
        const auto feedback_time = std::chrono::steady_clock::now();
        const auto previous_sequence = joint.last_sequence;
        const auto sequence_gap =
            state->update_sequence - previous_sequence;
        joint.last_sequence = state->update_sequence;
        joint.last_measured = robot::ti5::positionCountsToRadians(
            state->position_counts->value,
            joint.config.motor.encoder.counts_per_output_revolution);

        const double feedback_interval_seconds =
            std::chrono::duration<double>(
                feedback_time - previous_feedback_time)
                .count();
        if (!(feedback_interval_seconds > 0.0))
        {
            throw std::runtime_error(
                joint.config.name +
                " 的边界接管反馈时间间隔无效");
        }
        const double position_step =
            joint.last_measured - previous_measured;
        const double observed_speed =
            std::abs(position_step) / feedback_interval_seconds;
        maximum_observed_speed = std::max(
            maximum_observed_speed, observed_speed);

        if (joint.last_measured <
                std::min(original_position, capture_target) -
                    kDriverBoundaryCaptureOvershootRad ||
            joint.last_measured >
                std::max(original_position, capture_target) +
                    kDriverBoundaryCaptureOvershootRad)
        {
            throw std::runtime_error(
                joint.config.name +
                " 的边界接管反馈越过计划包络");
        }
        if (observed_speed >
            kMaximumDriverBoundaryCaptureSpeedRadPerSecond)
        {
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << joint.config.name
                    << " 的边界接管观测速度=" << observed_speed
                    << " rad/s，超过上限 "
                    << kMaximumDriverBoundaryCaptureSpeedRadPerSecond
                    << " rad/s；反馈 " << previous_measured
                    << " -> " << joint.last_measured
                    << " rad，位移=" << position_step
                    << " rad，实际 dt="
                    << feedback_interval_seconds
                    << " s，sequence_gap=" << sequence_gap;
            throw std::runtime_error(message.str());
        }
        previous_measured = joint.last_measured;
        previous_feedback_time = feedback_time;
    }

    const auto final_position = joint.motor->queryPosition();
    const auto final_status = queryDriverStatus(joint, mapping);
    if (!final_position ||
        std::abs(*final_position - capture_target) > 0.012 ||
        final_status.mode != 8 ||
        final_status.fault != 0)
    {
        throw std::runtime_error(
            joint.config.name +
            " 未完成驱动器边界接管，禁止继续方向测试");
    }

    joint.start_position = *final_position;
    joint.last_measured = *final_position;
    joint.driver_boundary_capture_command_sent = false;
    joint.driver_boundary_capture_target.reset();

    if (const auto final_state = joint.motor->latestState();
        final_state && final_state->update_sequence > joint.last_sequence)
    {
        joint.last_sequence = final_state->update_sequence;
    }

    robot::common::logger()->info(
        "{} 驱动器边界接管完成：mode=8、fault=0，最大观测速度 {:.6f} rad/s，当前位置 {:.6f} rad",
        joint.config.name,
        maximum_observed_speed,
        *final_position);
}

void runReadyProbe(JointRuntime &joint)
{
    robot::common::logger()->info(
        "{}：发送当前位置 CSP 就绪探测，不改变目标位置",
        joint.config.name);

    auto next_cycle = std::chrono::steady_clock::now();
    int fresh_cycles = 0;
    int stale_cycles = 0;
    for (int cycle = 0; cycle < kReadyProbeCycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error("收到中断，停止方向测试");
        }
        next_cycle += kControlPeriod;
        joint.motor->commandPositionCsp(joint.start_position);
        std::this_thread::sleep_until(next_cycle);

        const auto state = joint.motor->latestState();
        if (state && state->position_counts &&
            state->update_sequence > joint.last_sequence)
        {
            joint.last_sequence = state->update_sequence;
            joint.last_measured = robot::ti5::positionCountsToRadians(
                state->position_counts->value,
                joint.config.motor.encoder.counts_per_output_revolution);
            ++fresh_cycles;
            stale_cycles = 0;
        }
        else
        {
            ++stale_cycles;
        }
    }

    if (fresh_cycles < kReadyProbeCycles / 2 ||
        stale_cycles >= kMaximumStaleCycles)
    {
        throw std::runtime_error(
            joint.config.name +
            " CSP 反馈不稳定，未发送非零位移");
    }
    if (std::abs(joint.last_measured - joint.start_position) > 0.01)
    {
        throw std::runtime_error(
            joint.config.name + " 就绪探测后起点发生变化");
    }
}

class RawTerminal final
{
public:
    RawTerminal()
    {
        if (!::isatty(STDIN_FILENO))
        {
            throw std::runtime_error(
                "方向键测试必须在交互式终端中运行");
        }
        if (::tcgetattr(STDIN_FILENO, &original_) != 0)
        {
            throw std::runtime_error("读取终端属性失败");
        }

        termios raw = original_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_iflag &= static_cast<tcflag_t>(~(IXON | ICRNL));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0)
        {
            throw std::runtime_error("设置终端原始模式失败");
        }
        active_ = true;
    }

    ~RawTerminal()
    {
        if (active_)
        {
            static_cast<void>(::tcsetattr(
                STDIN_FILENO,
                TCSANOW,
                &original_));
        }
    }

    RawTerminal(const RawTerminal &) = delete;
    RawTerminal &operator=(const RawTerminal &) = delete;

private:
    termios original_{};
    bool active_{false};
};

enum class InputKey
{
    Positive,
    Negative,
    Quit,
};

class ArrowInput final
{
public:
    void readAvailable()
    {
        pollfd descriptor{
            STDIN_FILENO,
            POLLIN,
            0};
        while (::poll(&descriptor, 1, 0) > 0 &&
               (descriptor.revents & POLLIN) != 0)
        {
            std::array<unsigned char, 64> buffer{};
            const auto count = ::read(
                STDIN_FILENO,
                buffer.data(),
                buffer.size());
            if (count <= 0)
            {
                return;
            }
            input_.insert(
                input_.end(),
                buffer.begin(),
                buffer.begin() + count);
            descriptor.revents = 0;
        }
    }

    std::optional<InputKey> nextKey()
    {
        if (input_.empty())
        {
            return std::nullopt;
        }

        if (input_.front() == 'q' || input_.front() == 'Q')
        {
            input_.pop_front();
            return InputKey::Quit;
        }

        if (input_.front() == 0x1B)
        {
            if (input_.size() < 3)
            {
                return std::nullopt;
            }
            const auto second = input_[1];
            const auto third = input_[2];
            input_.pop_front();
            input_.pop_front();
            input_.pop_front();
            if ((second == '[' || second == 'O') && third == 'A')
            {
                return InputKey::Positive;
            }
            if ((second == '[' || second == 'O') && third == 'B')
            {
                return InputKey::Negative;
            }
            return std::nullopt;
        }

        input_.pop_front();
        return std::nullopt;
    }

private:
    std::deque<unsigned char> input_;
};

void holdBestEffort(JointRuntime &joint, const double requested_position)
{
    if (!std::isfinite(requested_position))
    {
        return;
    }
    const double hold_position = std::clamp(
        requested_position,
        joint.lower_limit,
        joint.upper_limit);
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 0; cycle < kHoldCycles; ++cycle)
    {
        next_cycle += kControlPeriod;
        try
        {
            joint.motor->commandPositionCsp(hold_position);
        }
        catch (const std::exception &)
        {
            break;
        }
        std::this_thread::sleep_until(next_cycle);
    }
}

void runInteractiveJog(
    JointRuntime &joint,
    const std::map<std::string, std::string> &mapping)
{
    try
    {
        captureAtDriverBoundary(joint, mapping);
        runReadyProbe(joint);
        double target = std::clamp(
            joint.last_measured,
            joint.lower_limit,
            joint.upper_limit);
        const double motion_lower =
            joint.lower_limit + kLimitMarginRad;
        const double motion_upper =
            joint.upper_limit - kLimitMarginRad;
        const double command_lower = motion_lower < motion_upper
                                         ? motion_lower
                                         : joint.lower_limit;
        const double command_upper = motion_lower < motion_upper
                                         ? motion_upper
                                         : joint.upper_limit;

        RawTerminal terminal;
        ArrowInput input;
        std::cout
            << "\n进入 " << joint.config.name
            << "（node " << joint.config.motor.node_id << "）\n"
            << "↑：正增量，↓：负增量；松开后约 150 ms 自动保持；q：退出当前电机测试并返回列表\n"
            << "当前目标位置：" << target << " rad\n"
            << std::flush;

        int direction = 0;
        auto last_arrow_event =
            std::chrono::steady_clock::now() - kKeyReleaseTimeout;
        auto next_cycle = std::chrono::steady_clock::now();
        std::uint64_t cycle = 0;
        int stale_cycles = 0;

        while (true)
        {
            if (stop_requested.load())
            {
                throw std::runtime_error("收到中断，停止方向测试");
            }

            next_cycle += kControlPeriod;
            input.readAvailable();
            while (const auto key = input.nextKey())
            {
                if (*key == InputKey::Quit)
                {
                    holdBestEffort(joint, target);
                    std::cout << "\n已停止当前电机测试。\n";
                    return;
                }
                direction = *key == InputKey::Positive ? 1 : -1;
                last_arrow_event = std::chrono::steady_clock::now();
            }

            const auto now = std::chrono::steady_clock::now();
            if (direction != 0 &&
                now - last_arrow_event > kKeyReleaseTimeout)
            {
                direction = 0;
            }

            if (direction > 0)
            {
                if (target < command_upper)
                {
                    target = std::min(
                        target + joint.delta_rad,
                        command_upper);
                }
                else
                {
                    direction = 0;
                }
            }
            else if (direction < 0)
            {
                if (target > command_lower)
                {
                    target = std::max(
                        target - joint.delta_rad,
                        command_lower);
                }
                else
                {
                    direction = 0;
                }
            }

            joint.motor->commandPositionCsp(target);
            std::this_thread::sleep_until(next_cycle);

            const auto state = joint.motor->latestState();
            if (!state || !state->position_counts ||
                state->update_sequence <= joint.last_sequence)
            {
                ++stale_cycles;
            }
            else
            {
                joint.last_sequence = state->update_sequence;
                joint.last_measured = robot::ti5::positionCountsToRadians(
                    state->position_counts->value,
                    joint.config.motor.encoder.counts_per_output_revolution);
                stale_cycles = 0;
            }

            if (stale_cycles >= kMaximumStaleCycles)
            {
                throw std::runtime_error(
                    "CSP 反馈超时：" + joint.config.name);
            }
            if (joint.last_measured <
                    joint.lower_limit - kAllowedOvershootRad ||
                joint.last_measured >
                    joint.upper_limit + kAllowedOvershootRad)
            {
                throw std::runtime_error(
                    "反馈位置超出有效限位：" + joint.config.name);
            }

            ++cycle;
            if (cycle % 20 == 0)
            {
                const char direction_text =
                    direction > 0 ? '+' : direction < 0 ? '-' : ' ';
                std::cout
                    << '\r'
                    << "direction=" << direction_text
                    << " target=" << target
                    << " measured=" << joint.last_measured
                    << " rad        "
                    << std::flush;
            }
        }
    }
    catch (...)
    {
        if (joint.driver_boundary_capture_target &&
            joint.driver_boundary_capture_command_sent)
        {
            const double hold_target =
                *joint.driver_boundary_capture_target;
            robot::common::logger()->warn(
                "{} 边界接管异常；为避免突然释放，继续向已下达的边界内目标 {:.6f} rad 发送保护性 HOLD {:.2f} 秒；不发送 STOP，后续方向点动已取消",
                joint.config.name,
                hold_target,
                static_cast<double>(kHoldCycles) *
                    std::chrono::duration<double>(kControlPeriod).count());
            holdBestEffort(joint, hold_target);
            try
            {
                const auto final_position =
                    joint.motor->queryPosition();
                const auto final_status =
                    queryDriverStatus(joint, mapping);
                if (final_position)
                {
                    robot::common::logger()->warn(
                        "{} 异常后保护性 HOLD 状态：位置 {:.6f} rad、mode={}、fault=0x{:08X}；程序退出后仍不得假定电机已释放",
                        joint.config.name,
                        *final_position,
                        final_status.mode,
                        static_cast<std::uint32_t>(final_status.fault));
                }
                else
                {
                    robot::common::logger()->error(
                        "{} 异常后保护性 HOLD 的最终位置查询失败；mode={}、fault=0x{:08X}",
                        joint.config.name,
                        final_status.mode,
                        static_cast<std::uint32_t>(final_status.fault));
                }
            }
            catch (const std::exception &status_error)
            {
                robot::common::logger()->error(
                    "{} 异常后保护性 HOLD 状态复核失败：{}；不得假定电机已释放",
                    joint.config.name,
                    status_error.what());
            }
        }
        else if (joint.driver_boundary_capture_target)
        {
            robot::common::logger()->warn(
                "{} 在边界接管目标发送前失败；未发送保护性 HOLD 或 STOP，后续方向点动已取消",
                joint.config.name);
        }
        else
        {
            holdBestEffort(joint, joint.last_measured);
        }
        throw;
    }
}

void printMotorList(const std::vector<JointRuntime> &joints)
{
    std::cout
        << "\n可测试电机（头部在前，然后是左臂和右臂的肩部到手腕）：\n";
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
        const auto &joint = joints[index];
        std::cout
            << "  " << (index + 1) << ". "
            << joint.config.name
            << " / " << joint.config.physical_name
            << " / node " << joint.config.motor.node_id
            << " / current=" << joint.last_measured << " rad";
        if (joint.driver_boundary_capture_target)
        {
            std::cout << " / 进入测试前先执行驱动器边界接管";
        }
        if (!joint.skip_reason.empty())
        {
            std::cout << " / 跳过：" << joint.skip_reason;
        }
        std::cout << '\n';
    }
}

std::optional<std::size_t> promptMotorSelection(
    const std::vector<JointRuntime> &joints)
{
    while (true)
    {
        printMotorList(joints);
        std::cout << "输入电机序号进入测试，q 退出：" << std::flush;
        std::string answer;
        if (!std::getline(std::cin, answer))
        {
            throw std::runtime_error("无法读取电机序号");
        }
        if (answer == "q" || answer == "Q")
        {
            return std::nullopt;
        }

        std::size_t parsed_end = 0;
        unsigned long number = 0;
        try
        {
            number = std::stoul(answer, &parsed_end);
        }
        catch (const std::exception &)
        {
            number = 0;
        }
        if (number == 0 || parsed_end != answer.size() ||
            number > joints.size())
        {
            std::cout << "序号无效，请重新输入。\n";
            continue;
        }
        const auto index = static_cast<std::size_t>(number - 1);
        if (!joints[index].skip_reason.empty())
        {
            std::cout << "该电机当前不可测试："
                      << joints[index].skip_reason << "\n";
            continue;
        }
        return index;
    }
}

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try
    {
        const auto options = parseOptions(argc, argv);
        requireExplicitConfirmation();
        SingleProcessLock process_lock;
        requireExclusiveController();

        robot::common::logger()->info(
            "TI5 头部与双臂电机方向标注测试：启动并拉起本体 CAN");

        const auto robot_config = robot::ti5::loadRobotConfig(
            configPath("robot.yaml"));
        const auto can_config = robot::ti5::loadCanConfig(
            configPath("can.yaml"));
        const auto safety_limits = loadSafetyLimits(
            configPath("safety.yaml"));
        const auto ordered_joints = orderedDirectionJoints(robot_config);

        const auto candidate_interfaces = prepareBodyCan(can_config);
        const auto mapping = discoverDirectionBuses(
            robot_config,
            can_config,
            candidate_interfaces);

        std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> buses;
        for (const auto &[bus_name, interface_name] : mapping)
        {
            robot::common::logger()->info(
                "方向测试总线映射：{} -> {}",
                bus_name,
                interface_name);
            buses.emplace(
                bus_name,
                std::make_unique<robot::ti5::CanBus>(interface_name));
        }

        auto joints = buildRuntimes(
            ordered_joints,
            safety_limits,
            buses,
            options.delta_rad);

        std::cout
            << "\n准备完成；程序测试头部 3 轴和双臂 14 轴，不测试腰部和折叠轴。\n"
            << "输入编号选择电机；进入后 ↑/↓ 为正负增量，q 返回列表。\n"
            << "程序不会写零位、不会自动发送 STOP/disable。\n";
        while (true)
        {
            const auto selection = promptMotorSelection(joints);
            if (!selection)
            {
                break;
            }
            robot::common::logger()->info(
                "开始交互点动 {}（CAN node {}）",
                joints[*selection].config.name,
                joints[*selection].config.motor.node_id);
            runInteractiveJog(joints[*selection], mapping);
        }

        robot::common::logger()->info(
            "方向测试结束；未写入零位或方向配置");
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "方向测试失败：{}",
            error.what());
        return 1;
    }
}
