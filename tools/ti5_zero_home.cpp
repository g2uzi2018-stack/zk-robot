#include "can/can_frame.hpp"
#include "can/can_interface_manager.hpp"
#include "can/socket_can.hpp"
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
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{

using namespace std::chrono_literals;

constexpr auto kControlPeriod = 10ms;
constexpr auto kInterFrameGap = 50us;
constexpr int kReadyProbeCycles = 40;
constexpr int kDriverBoundaryCaptureCycles = 250;
constexpr int kFinalHoldCycles = 100;
constexpr int kMaximumStaleCycles = 3;
constexpr double kPositionAgreementRad = 0.01;
constexpr double kLimitMarginRad = 0.02;
constexpr double kAllowedPathOvershootRad = 0.03;
constexpr double kFinalToleranceRad = 0.012;
constexpr double kDriverBoundaryCaptureOvershootRad = 0.01;
constexpr double kMaximumVelocityRadPerSecond = 0.15;
constexpr double kLoweringVelocityRadPerSecond = 0.10;
constexpr double kMaximumAutomaticTravelRad = 0.50;
constexpr double kMaximumShoulderRollRecoveryTravelRad = 1.62;
constexpr double kMaximumDriverBoundaryCaptureRad = 0.12;
constexpr double kDriverBoundaryCaptureMarginRad = 0.005;
constexpr double kMaximumDriverBoundaryCaptureSpeedRadPerSecond = 0.20;
constexpr double kMinimumMoveSeconds = 2.0;
constexpr int kIntermediateHoldCycles = 50;

const std::set<std::string> kControlledBuses{
    "head",
    "left_arm",
    "right_arm",
};

// 2026-08-23 菜单 2 完成自然下垂后，只读采样 3 秒、每轴 60 帧；
// 所有轴 mode=0、fault=0，且每轴 60 帧完全稳定。这里保存 CAN 原始
// 输出端 counts，不经过 kinematics.yaml。完整下垂点只作为缓降插值参考，
// 不作为肩横滚恢复包络；程序绝不以 CSP 命令完整下垂点，只命令到
// q0->q_down 的第二中间点。
const std::map<std::string, std::int32_t> kNaturalDroopCounts{
    {"left_shoulder_pitch", -3979},
    {"left_shoulder_roll", -65990},
    {"left_shoulder_yaw", 21},
    {"left_elbow_yaw", -77},
    {"left_wrist_pitch", -84},
    {"left_wrist_yaw", -425},
    {"left_wrist_roll", 5106},
    {"right_shoulder_pitch", 744},
    {"right_shoulder_roll", 62706},
    {"right_shoulder_yaw", -26},
    {"right_elbow_yaw", 1293},
    {"right_wrist_pitch", -47},
    {"right_wrist_yaw", 100},
    {"right_wrist_roll", -15},
};

std::atomic<bool> stop_requested{false};

void signalHandler(int)
{
    stop_requested.store(true);
}

struct Options
{
    bool dry_run{false};
    bool bring_up{true};
};

enum class MenuAction
{
    ZeroHome,
    StopArms,
    HoldCurrent,
};

struct SafetyLimit
{
    double minimum{0.0};
    double maximum{0.0};
    bool verified_on_robot{false};
};

struct DriverStatus
{
    std::uint32_t mode{0};
    std::uint32_t fault{0};
};

struct FeedbackTracker
{
    std::uint64_t last_sequence{0};
    int fresh_cycles{0};
    int stale_cycles{0};
    double measured_position{0.0};
};

struct JointRuntime
{
    robot::ti5::PhysicalJointConfig config;
    SafetyLimit safety;
    DriverStatus status;
    std::unique_ptr<robot::ti5::CanMotor> motor;
    robot::ti5::DriverPositionLimits driver_limits;
    double start_position{0.0};
    double last_commanded{0.0};
    std::optional<double> driver_boundary_capture_target;
    FeedbackTracker feedback;
};

struct LoweringPlan
{
    std::vector<double> first_intermediate;
    std::vector<double> second_intermediate;
    std::vector<double> natural_droop_reference;
};

class SingleProcessLock final
{
public:
    SingleProcessLock()
    {
        constexpr const char *lock_path =
            "/tmp/zk_robot_ti5_motion.lock";

        // fs.protected_regular may reject root opening another user's file in
        // sticky /tmp when O_CREAT is present, even if the file is 0666.
        // Open an existing inode without O_CREAT; only create on ENOENT.
        fd_ = ::open(lock_path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0 && errno == ENOENT)
        {
            fd_ = ::open(lock_path,
                         O_CREAT | O_EXCL | O_RDONLY |
                             O_CLOEXEC | O_NOFOLLOW,
                         0666);
            if (fd_ < 0 && errno == EEXIST)
            {
                fd_ = ::open(
                    lock_path,
                    O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
            }
        }
        if (fd_ < 0)
        {
            const int saved_errno = errno;
            throw std::runtime_error(
                "无法打开 TI5 实机运动互斥锁：" +
                std::string{std::strerror(saved_errno)});
        }
        // main 和本测试可能分别以 kuang/root 身份运行。锁只承载 flock，
        // 不保存数据；放宽权限可避免 /tmp 上的 root-squash 所有权残留。
        static_cast<void>(::fchmod(fd_, 0666));
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
        {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("另一个 zk_robot 实机运动程序正在运行");
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

void printUsage(const char *program)
{
    std::cout
        << "用法：\n"
        << "  " << program << " [--dry-run] [--no-bring-up]\n\n"
        << "默认回零范围：头部 3 轴 + 左臂 7 轴 + 右臂 7 轴。\n"
        << "固定排除：waist_yaw、Fold_P1/P2/P3/R 和傲意手。\n"
        << "--dry-run       只打印计划，不打开 CAN，不发送任何帧\n"
        << "--no-bring-up   不重配 CAN；要求本体接口已经 UP/1 Mbps\n";
}

Options parseOptions(const int argc, char **argv)
{
    Options options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument{argv[index]};
        if (argument == "--dry-run")
        {
            options.dry_run = true;
        }
        else if (argument == "--no-bring-up")
        {
            options.bring_up = false;
        }
        else if (argument == "--help" || argument == "-h")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        else
        {
            throw std::invalid_argument("未知参数：" + argument);
        }
    }
    return options;
}

std::map<std::string, SafetyLimit> loadSafetyLimits(
    const std::filesystem::path &path)
{
    const YAML::Node root = YAML::LoadFile(path.string());
    const YAML::Node limits = root["position_limits"];
    if (!limits || !limits.IsMap())
    {
        throw std::runtime_error("safety.yaml 缺少 position_limits");
    }

    std::map<std::string, SafetyLimit> result;
    for (const auto &entry : limits)
    {
        const std::string name = entry.first.as<std::string>();
        const YAML::Node value = entry.second;
        SafetyLimit limit{
            value["min"].as<double>(),
            value["max"].as<double>(),
            value["verified_on_robot"].as<bool>(false),
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
            !std::all_of(pid_text.begin(), pid_text.end(),
                         [](const unsigned char value)
                         { return value >= '0' && value <= '9'; }))
        {
            continue;
        }
        if (std::stol(pid_text) == self)
        {
            continue;
        }

        std::ifstream comm{entry->path() / "comm"};
        std::string process_name;
        if (std::getline(comm, process_name) && process_name == expected_name)
        {
            return true;
        }
    }
    return false;
}

void requireExclusiveController()
{
    static const std::array<const char *, 4> forbidden{
        "Ti5Control",
        "joint_manager",
        "ti5_follow_arm",
        "robot",
    };
    for (const char *name : forbidden)
    {
        if (processNamedIsRunning(name))
        {
            throw std::runtime_error(
                std::string{"检测到其他控制程序："} + name +
                "；请先关闭，回零程序必须独占本体 CAN");
        }
    }
}

void requireEnvironmentConfirmation()
{
    const char *value = std::getenv("ZK_ROBOT_CONFIRM_TI5_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "缺少 ZK_ROBOT_CONFIRM_TI5_TEST=YES；未打开 CAN");
    }
}

MenuAction promptMenuAction()
{
    std::cout
        << "\n请选择本次动作：\n"
        << "  1 = 头部和双臂回到 CAN 电机零点并保持\n"
        << "  2 = 双臂缓慢下放到自然下垂参考的第二中间点，再发送 0x02 STOP\n"
        << "      到达后会暂停，等操作者托稳并二次确认才 STOP\n"
        << "  3 = 头部和双臂在当前位置建立 CSP HOLD\n"
        << "  q = 不打开 CAN，直接退出\n"
        << "请输入 1、2、3 或 q：" << std::flush;
    std::string answer;
    if (!std::getline(std::cin, answer))
    {
        throw std::runtime_error("无法读取菜单选择；未打开 CAN");
    }
    if (answer == "1")
    {
        return MenuAction::ZeroHome;
    }
    if (answer == "2")
    {
        return MenuAction::StopArms;
    }
    if (answer == "3")
    {
        return MenuAction::HoldCurrent;
    }
    if (answer == "q" || answer == "Q")
    {
        std::cout << "已退出；未打开 CAN。\n";
        std::exit(0);
    }
    throw std::runtime_error("菜单只能输入 1、2、3 或 q；未打开 CAN");
}

void requireYes(const std::string &message)
{
    std::cout << "\n" << message
              << "\n确认请输入 y 或 Y；其他输入取消：" << std::flush;
    std::string answer;
    if (!std::getline(std::cin, answer) ||
        !(answer == "y" || answer == "Y"))
    {
        throw std::runtime_error("操作者取消；未发送控制帧");
    }
}

void requireFinalStopConfirmation()
{
    std::cout
        << "\n双臂现在由 CSP 保持在第二中间点，尚未发送 0x02 STOP。\n"
        << "请先用双手或机械支撑可靠承托双臂；STOP 后程序不能限制剩余重力下落。\n"
        << "确认已经托稳后请输入 y 或 Y；输入其他内容将保留当前 HOLD 并取消 STOP："
        << std::flush;
    std::string answer;
    if (!std::getline(std::cin, answer) ||
        !(answer == "y" || answer == "Y"))
    {
        throw std::runtime_error(
            "操作者取消最终 STOP；双臂保留第二中间点 CSP HOLD，未发送 0x02");
    }
}

double quinticBlend(const double ratio)
{
    const double t = std::clamp(ratio, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

double radiansToDegrees(const double radians)
{
    constexpr double pi = 3.14159265358979323846;
    return radians * 180.0 / pi;
}

bool isShoulderRoll(const std::string &joint_name)
{
    return joint_name == "left_shoulder_roll" ||
           joint_name == "right_shoulder_roll";
}

double maximumAutomaticTravelForJoint(const std::string &joint_name)
{
    return isShoulderRoll(joint_name)
               ? kMaximumShoulderRollRecoveryTravelRad
               : kMaximumAutomaticTravelRad;
}

double naturalDroopReferenceForJoint(
    const robot::ti5::PhysicalJointConfig &joint)
{
    const auto reference = kNaturalDroopCounts.find(joint.name);
    if (reference == kNaturalDroopCounts.end())
    {
        throw std::runtime_error("缺少自然下垂参考值：" + joint.name);
    }
    return robot::ti5::positionCountsToRadians(
        reference->second,
        joint.motor.encoder.counts_per_output_revolution);
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

std::vector<robot::ti5::PhysicalJointConfig> selectJoints(
    const robot::ti5::Ti5RobotConfig &robot_config)
{
    std::vector<robot::ti5::PhysicalJointConfig> result;
    for (const auto &joint : robot_config.joints)
    {
        if (kControlledBuses.count(joint.bus) != 0)
        {
            result.push_back(joint);
        }
    }
    if (result.size() != 17)
    {
        throw std::runtime_error(
            "回零计划必须正好包含头部 3 轴和双臂 14 轴，当前为 " +
            std::to_string(result.size()) + " 轴");
    }
    return result;
}

std::vector<robot::ti5::PhysicalJointConfig> selectArmJoints(
    const std::vector<robot::ti5::PhysicalJointConfig> &joints)
{
    std::vector<robot::ti5::PhysicalJointConfig> result;
    for (const auto &joint : joints)
    {
        if (joint.bus == "left_arm" || joint.bus == "right_arm")
        {
            result.push_back(joint);
        }
    }
    if (result.size() != 14)
    {
        throw std::runtime_error(
            "双臂计划必须正好包含 14 个关节，当前为 " +
            std::to_string(result.size()) + " 轴");
    }
    return result;
}

void printPlan(
    const std::vector<robot::ti5::PhysicalJointConfig> &joints,
    const std::map<std::string, SafetyLimit> &limits,
    const Options &options)
{
    std::cout
        << "TI5 T170C 独立实机测试菜单\n"
        << "动作：电机零点回零 / 双臂两段缓降后 STOP / 当前位置 HOLD\n"
        << "排除：waist_yaw、Fold_P1/P2/P3/R、傲意手\n"
        << "CAN：" << (options.bring_up ? "自动拉起四路本体 CAN" : "使用已拉起的 CAN")
        << "\n"
        << "普通关节最大自动回零距离：" << kMaximumAutomaticTravelRad
        << " rad（" << radiansToDegrees(kMaximumAutomaticTravelRad) << "°）\n"
        << "肩横滚受控恢复最大距离："
        << kMaximumShoulderRollRecoveryTravelRad
        << " rad（"
        << radiansToDegrees(kMaximumShoulderRollRecoveryTravelRad)
        << "°）\n"
        << "轨迹峰值速度上限：" << kMaximumVelocityRadPerSecond
        << " rad/s\n\n";

    bool has_unverified_limit = false;
    int index = 1;
    for (const auto &joint : joints)
    {
        const auto limit = limits.find(joint.name);
        if (limit == limits.end())
        {
            throw std::runtime_error("缺少软件限位：" + joint.name);
        }
        has_unverified_limit = has_unverified_limit ||
                               !limit->second.verified_on_robot;
        std::cout << std::right << std::setw(2) << index++ << ". "
                  << std::left << std::setw(27) << joint.name
                  << " / " << std::setw(13) << joint.physical_name
                  << " / " << std::setw(10) << joint.bus
                  << " / node " << joint.motor.node_id << "\n";
    }
    if (has_unverified_limit)
    {
        std::cout
            << "\n警告：safety.yaml 的这些关节仍标记为未经完整实机限位验证。\n";
    }
    std::cout
        << "\n菜单 1：17 轴回到 CAN 电机角 0 并保持\n"
        << "菜单 2：双臂经两个中间点缓慢下放，托稳确认后在第二中间点 STOP\n"
        << "菜单 3：17 轴在当前位置建立 CSP HOLD\n";
}

std::vector<std::string> prepareBodyInterfaces(
    const Options &options,
    const robot::ti5::CanConfig &can_config,
    const robot::can::CanInterfaceManager &manager)
{
    const auto all_interfaces = manager.enumerate(
        can_config.socketcan.interface_regex);
    if (all_interfaces.empty())
    {
        throw std::runtime_error("没有找到 SocketCAN 接口");
    }

    const auto &selector = can_config.socketcan.body_adapter;
    const auto selected = manager.selectAdapter(
        all_interfaces,
        robot::can::CanAdapterSelector{
            selector.selector,
            selector.value,
            selector.expected_channels});

    const robot::can::CanInterfaceSettings settings{
        can_config.socketcan.bitrate,
        static_cast<std::uint32_t>(can_config.socketcan.restart_ms.count()),
        can_config.socketcan.reconfigure_wait,
        can_config.socketcan.startup_wait,
        can_config.socketcan.validate_bitrate};

    std::vector<std::string> result;
    result.reserve(selected.size());
    for (const auto &interface : selected)
    {
        robot::can::CanInterfaceInfo ready;
        if (options.bring_up)
        {
            ready = manager.prepare(interface.name, settings);
        }
        else
        {
            ready = manager.inspect(interface.name);
            if (!ready.up || !ready.bitrate ||
                *ready.bitrate != can_config.socketcan.bitrate)
            {
                throw std::runtime_error(
                    interface.name +
                    " 尚未以 1 Mbps 拉起；请移除 --no-bring-up 后使用 sudo 运行");
            }
        }
        robot::common::logger()->info(
            "CAN {} 已就绪：bitrate={}，restart-ms={}",
            ready.name,
            ready.bitrate ? std::to_string(*ready.bitrate) : "未知",
            ready.restart_ms ? std::to_string(*ready.restart_ms) : "未知");
        result.push_back(ready.name);
    }
    return result;
}

std::map<std::string, std::string> discoverBusInterfaces(
    const std::vector<robot::ti5::LogicalCanBus> &logical_buses,
    const robot::ti5::CanConfig &can_config,
    const std::vector<std::string> &candidate_interfaces)
{
    const robot::ti5::CanDiscovery discovery;
    const auto result = discovery.discover(
        logical_buses,
        robot::ti5::makeDiscoveryOptions(can_config),
        candidate_interfaces);
    if (!result.success)
    {
        throw std::runtime_error("TI5 本体 CAN 探测失败");
    }

    std::map<std::string, std::string> mapping;
    for (const auto &bus : result.logical_buses)
    {
        if (!bus.complete || !bus.interface_name)
        {
            throw std::runtime_error("逻辑 CAN 总线节点不完整：" + bus.bus_name);
        }
        mapping.emplace(bus.bus_name, *bus.interface_name);
    }
    for (const auto &required : kControlledBuses)
    {
        if (mapping.count(required) == 0)
        {
            throw std::runtime_error("没有发现回零所需逻辑总线：" + required);
        }
    }
    return mapping;
}

std::map<std::string, DriverStatus> queryDriverStatuses(
    const std::vector<robot::ti5::PhysicalJointConfig> &joints,
    std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> &buses)
{
    std::map<std::string, DriverStatus> result;
    for (const auto &joint : joints)
    {
        const auto bus = buses.find(joint.bus);
        if (bus == buses.end() || !bus->second)
        {
            throw std::runtime_error(
                "没有为关节创建 CAN 总线：" + joint.name);
        }
        robot::ti5::CanMotor motor{joint.motor, *bus->second};
        const auto status = motor.queryDriverStatus();
        if (!status)
        {
            throw std::runtime_error(
                joint.name + " 的模式或故障查询失败");
        }
        result.emplace(
            joint.name,
            DriverStatus{
                status->run_mode,
                status->fault_bits});
    }
    return result;
}

DriverStatus queryDriverStatus(JointRuntime &joint)
{
    const auto status = joint.motor->queryDriverStatus();
    if (!status)
    {
        throw std::runtime_error(
            joint.config.name + " 的模式或故障查询失败");
    }
    return DriverStatus{status->run_mode, status->fault_bits};
}

void sendStopToBothArms(
    const std::vector<robot::ti5::PhysicalJointConfig> &joints,
    std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> &buses,
    const std::map<std::string, std::string> &mapping)
{
    static_cast<void>(mapping);
    auto arm_joints = selectArmJoints(joints);

    std::sort(
        arm_joints.begin(),
        arm_joints.end(),
        [](const auto &left, const auto &right)
        {
            if (left.bus != right.bus)
            {
                return left.bus < right.bus;
            }
            return left.motor.node_id > right.motor.node_id;
        });

    robot::common::logger()->warn(
        "开始双臂 0x02 STOP：按每条总线从末端到肩部发送，头部不发送 STOP");
    for (const auto &joint : arm_joints)
    {
        robot::can::CanFrame frame{};
        frame.id = joint.motor.node_id;
        frame.data_length = 1;
        frame.data[0] = 0x02;
        buses.at(joint.bus)->send(frame);
        std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(250ms);

    const auto statuses = queryDriverStatuses(arm_joints, buses);
    std::vector<std::string> failures;
    for (const auto &joint : arm_joints)
    {
        const auto &status = statuses.at(joint.name);
        if (status.mode != 0 || status.fault != 0)
        {
            std::ostringstream message;
            message << joint.name << ": mode=" << status.mode
                    << ", fault=0x" << std::hex << std::uppercase
                    << static_cast<std::uint32_t>(status.fault);
            failures.push_back(message.str());
        }
    }
    if (!failures.empty())
    {
        std::ostringstream message;
        message << "双臂 STOP 已发送，但以下节点状态验证失败：";
        for (const auto &failure : failures)
        {
            message << "\n  " << failure;
        }
        throw std::runtime_error(message.str());
    }
    robot::common::logger()->info(
        "双臂 14 个节点均已确认 mode=0、fault=0；头部状态未改变");
    robot::common::logger()->warn(
        "mode=0 只表示 STOP 运行模式，不证明去使能、抱闸释放或一定自然下垂");
}

std::vector<JointRuntime> buildCspRuntimes(
    const std::vector<robot::ti5::PhysicalJointConfig> &selected_joints,
    const std::map<std::string, SafetyLimit> &limits,
    const std::map<std::string, DriverStatus> &statuses,
    std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> &buses,
    const bool require_zero_reachable)
{
    std::vector<JointRuntime> joints;
    joints.reserve(selected_joints.size());
    for (const auto &config : selected_joints)
    {
        const auto safety = limits.find(config.name);
        if (safety == limits.end())
        {
            throw std::runtime_error("缺少软件限位：" + config.name);
        }
        const auto status = statuses.find(config.name);
        if (status == statuses.end())
        {
            throw std::runtime_error("缺少驱动器状态：" + config.name);
        }
        if (status->second.mode != 0 && status->second.mode != 8)
        {
            throw std::runtime_error(
                config.name + " 当前 mode=" +
                std::to_string(status->second.mode) +
                "，只允许从 mode=0 或 mode=8 建立 CSP");
        }
        if (status->second.fault != 0)
        {
            std::ostringstream message;
            message << config.name << " fault=0x"
                    << std::hex << std::uppercase
                    << static_cast<std::uint32_t>(status->second.fault)
                    << "，禁止建立 CSP";
            throw std::runtime_error(message.str());
        }

        auto motor = std::make_unique<robot::ti5::CanMotor>(
            config.motor,
            *buses.at(config.bus));
        const auto position = motor->queryPosition();
        const auto driver_limits = motor->queryPositionLimits();
        if (!position || !driver_limits)
        {
            throw std::runtime_error(
                config.name + " 的当前位置或 0x1A/0x1B 查询失败");
        }
        const bool outside_host_limit =
            *position < safety->second.minimum ||
            *position > safety->second.maximum;
        const double distance_outside_driver = std::max(
            {driver_limits->minimum_rad - *position,
             *position - driver_limits->maximum_rad,
             0.0});
        const bool inside_shoulder_recovery_corridor =
            require_zero_reachable &&
            isKnownShoulderRecoveryCorridor(
                config, *driver_limits, *position);
        std::optional<double> shoulder_capture_target;
        double shoulder_capture_distance = 0.0;
        bool shoulder_capture_target_inside_host_limit = false;
        if (inside_shoulder_recovery_corridor)
        {
            const double capture_lower =
                driver_limits->minimum_rad +
                kDriverBoundaryCaptureMarginRad;
            const double capture_upper =
                driver_limits->maximum_rad -
                kDriverBoundaryCaptureMarginRad;
            if (capture_lower >= capture_upper)
            {
                throw std::runtime_error(
                    config.name +
                    " 的驱动器目标范围过窄，无法建立边界内接管目标");
            }
            shoulder_capture_target = std::clamp(
                *position, capture_lower, capture_upper);
            shoulder_capture_distance = std::abs(
                *shoulder_capture_target - *position);
            shoulder_capture_target_inside_host_limit =
                *shoulder_capture_target >= safety->second.minimum &&
                *shoulder_capture_target <= safety->second.maximum;
        }
        const bool known_shoulder_recovery =
            inside_shoulder_recovery_corridor &&
            shoulder_capture_target_inside_host_limit &&
            shoulder_capture_distance <=
                kMaximumDriverBoundaryCaptureRad;
        if (outside_host_limit && !known_shoulder_recovery)
        {
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << config.name << " 当前电机角=" << *position
                    << " rad，位于 safety.yaml 之外；禁止建立 CSP HOLD";
            if (require_zero_reachable && isShoulderRoll(config.name))
            {
                message << "。肩横滚恢复预检失败：允许的被动下垂包络为 ±"
                        << kMaximumShoulderRollRecoveryTravelRad << " rad";
                if (shoulder_capture_target)
                {
                    message << "，最近边界内目标="
                            << *shoulder_capture_target
                            << " rad，实际接管距离="
                            << shoulder_capture_distance
                            << " rad（上限 "
                            << kMaximumDriverBoundaryCaptureRad << " rad）";
                    if (!shoulder_capture_target_inside_host_limit)
                    {
                        message << "，且接管目标位于 safety.yaml 之外";
                    }
                }
                message << "；禁止边界接管";
            }
            throw std::runtime_error(message.str());
        }
        if (outside_host_limit && known_shoulder_recovery)
        {
            robot::common::logger()->warn(
                "{} 当前 {:.6f} rad 位于主机软限位外；仅允许沿单调轨迹向电机零点恢复",
                config.name,
                *position);
        }
        if (*position < driver_limits->minimum_rad ||
            *position > driver_limits->maximum_rad)
        {
            if (!known_shoulder_recovery)
            {
                std::ostringstream message;
                message
                    << std::fixed << std::setprecision(6)
                    << config.name << " 当前电机角=" << *position
                    << " rad，位于驱动器 0x1A/0x1B 之外；禁止建立 CSP HOLD";
                if (require_zero_reachable && isShoulderRoll(config.name))
                {
                    message << "；被动下垂包络为 ±"
                            << kMaximumShoulderRollRecoveryTravelRad << " rad";
                    if (shoulder_capture_target)
                    {
                        message << "，实际边界接管距离="
                                << shoulder_capture_distance
                                << " rad（上限 "
                                << kMaximumDriverBoundaryCaptureRad << " rad）";
                    }
                }
                throw std::runtime_error(message.str());
            }
            robot::common::logger()->warn(
                "{} 超出驱动器目标范围 {:.6f} rad；回零前先执行边界内接管",
                config.name,
                distance_outside_driver);
        }

        if (require_zero_reachable)
        {
            if (0.0 < safety->second.minimum + kLimitMarginRad ||
                0.0 > safety->second.maximum - kLimitMarginRad)
            {
                throw std::runtime_error(
                    config.name + " 的电机零点不在软件限位安全余量内");
            }
            if (0.0 < driver_limits->minimum_rad + kLimitMarginRad ||
                0.0 > driver_limits->maximum_rad - kLimitMarginRad)
            {
                throw std::runtime_error(
                    config.name + " 的电机零点不在驱动器目标范围安全余量内");
            }
            const double maximum_automatic_travel =
                maximumAutomaticTravelForJoint(config.name);
            if (std::abs(*position) > maximum_automatic_travel)
            {
                std::ostringstream message;
                message << std::fixed << std::setprecision(6)
                        << config.name << " 距离电机零点 "
                        << std::abs(*position) << " rad，超过自动回零上限 "
                        << maximum_automatic_travel << " rad；";
                if (isShoulderRoll(config.name))
                {
                    message
                        << "请保持 mode=0，可靠托住并手动抬至该范围内后重试";
                }
                else
                {
                    message
                        << "请先用已验证的上位机恢复到零点附近";
                }
                throw std::runtime_error(message.str());
            }
        }

        const auto state = motor->latestState();
        if (!state || !state->position_counts)
        {
            throw std::runtime_error(config.name + " 没有初始位置状态");
        }
        const double state_position =
            robot::ti5::positionCountsToRadians(
                state->position_counts->value,
                config.motor.encoder.counts_per_output_revolution);
        if (std::abs(state_position - *position) > kPositionAgreementRad)
        {
            throw std::runtime_error(
                config.name + " 的初始位置查询结果不一致");
        }

        JointRuntime runtime;
        runtime.config = config;
        runtime.safety = safety->second;
        runtime.status = status->second;
        runtime.motor = std::move(motor);
        runtime.driver_limits = *driver_limits;
        runtime.start_position = *position;
        runtime.last_commanded = *position;
        if (known_shoulder_recovery && shoulder_capture_target &&
            shoulder_capture_distance > 1e-6)
        {
            runtime.driver_boundary_capture_target =
                *shoulder_capture_target;
        }
        runtime.feedback.last_sequence = state->csp_update_sequence;
        runtime.feedback.measured_position = state_position;
        joints.push_back(std::move(runtime));
    }
    return joints;
}

void validateLoweringTarget(
    const JointRuntime &joint,
    const double target,
    const std::string &target_name)
{
    if (target < joint.safety.minimum + kLimitMarginRad ||
        target > joint.safety.maximum - kLimitMarginRad)
    {
        std::ostringstream message;
        message << std::fixed << std::setprecision(6)
                << joint.config.name << " 的" << target_name
                << "=" << target
                << " rad 不在 safety.yaml 限位的 0.02 rad 安全余量内";
        throw std::runtime_error(message.str());
    }
    if (target < joint.driver_limits.minimum_rad + kLimitMarginRad ||
        target > joint.driver_limits.maximum_rad - kLimitMarginRad)
    {
        std::ostringstream message;
        message << std::fixed << std::setprecision(6)
                << joint.config.name << " 的" << target_name
                << "=" << target
                << " rad 不在驱动器 0x1A/0x1B 的 0.02 rad 安全余量内";
        throw std::runtime_error(message.str());
    }
}

LoweringPlan buildLoweringPlan(const std::vector<JointRuntime> &joints)
{
    LoweringPlan plan;
    plan.first_intermediate.reserve(joints.size());
    plan.second_intermediate.reserve(joints.size());
    plan.natural_droop_reference.reserve(joints.size());

    for (const auto &joint : joints)
    {
        const double natural_droop =
            naturalDroopReferenceForJoint(joint.config);
        const double first = joint.start_position +
                             (natural_droop - joint.start_position) / 3.0;
        const double second = joint.start_position +
                              2.0 * (natural_droop - joint.start_position) /
                                  3.0;

        // 完整自然下垂参考可能超出命令限位，因此只检查并命令两个中间点。
        validateLoweringTarget(joint, first, "第一中间点");
        validateLoweringTarget(joint, second, "第二中间点");
        plan.first_intermediate.push_back(first);
        plan.second_intermediate.push_back(second);
        plan.natural_droop_reference.push_back(natural_droop);
    }
    return plan;
}

void printLoweringPlan(
    const std::vector<JointRuntime> &joints,
    const LoweringPlan &plan)
{
    std::cout
        << "\n双臂两段缓降预检通过（尚未发送 0x44）：\n"
        << "关节                         起点°      中间点1°      中间点2°   完整下垂参考°\n";
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
        std::cout << std::left << std::setw(29)
                  << joints[index].config.name
                  << std::right << std::fixed << std::setprecision(3)
                  << std::setw(10)
                  << radiansToDegrees(joints[index].start_position)
                  << std::setw(14)
                  << radiansToDegrees(plan.first_intermediate[index])
                  << std::setw(15)
                  << radiansToDegrees(plan.second_intermediate[index])
                  << std::setw(17)
                  << radiansToDegrees(plan.natural_droop_reference[index])
                  << "\n";
    }
    std::cout
        << "完整下垂列只是只读采样参考，程序不会把它作为 CSP 目标。\n"
        << "第二中间点约位于本次起点到完整下垂参考的 2/3 处。\n";
}

void printCspPreflight(
    const std::vector<JointRuntime> &joints,
    const std::string &title)
{
    std::cout
        << "\n" << title << "（尚未发送 0x44）：\n"
        << "关节                         起点 motor_rad      起点角度       mode\n";
    for (const auto &joint : joints)
    {
        std::cout << std::left << std::setw(29) << joint.config.name
                  << std::right << std::fixed << std::setprecision(6)
                  << std::setw(14) << joint.start_position
                  << std::setw(13) << radiansToDegrees(joint.start_position)
                  << "°"
                  << std::setw(8) << joint.status.mode << "\n";
        if (joint.driver_boundary_capture_target)
        {
            std::cout
                << "  -> 回零前先接管到驱动器边界内 "
                << std::fixed << std::setprecision(6)
                << *joint.driver_boundary_capture_target
                << " rad（"
                << radiansToDegrees(
                       *joint.driver_boundary_capture_target)
                << "°）\n";
        }
    }
}

void sendTargets(std::vector<JointRuntime> &joints)
{
    for (auto &joint : joints)
    {
        joint.motor->commandPositionCsp(joint.last_commanded);
        std::this_thread::sleep_for(kInterFrameGap);
    }
}

double updateFeedback(JointRuntime &joint)
{
    const auto state = joint.motor->latestState();
    if (!state || !state->position_counts)
    {
        ++joint.feedback.stale_cycles;
    }
    else if (state->csp_update_sequence <= joint.feedback.last_sequence)
    {
        ++joint.feedback.stale_cycles;
    }
    else
    {
        joint.feedback.last_sequence = state->csp_update_sequence;
        ++joint.feedback.fresh_cycles;
        joint.feedback.stale_cycles = 0;
        joint.feedback.measured_position =
            robot::ti5::positionCountsToRadians(
                state->position_counts->value,
                joint.config.motor.encoder.counts_per_output_revolution);
    }

    if (joint.feedback.stale_cycles >= kMaximumStaleCycles)
    {
        throw std::runtime_error(
            joint.config.name + " 的 CSP 反馈连续失效");
    }
    return joint.feedback.measured_position;
}

void captureShouldersAtDriverBoundary(
    std::vector<JointRuntime> &joints)
{
    for (auto &joint : joints)
    {
        if (!joint.driver_boundary_capture_target)
        {
            continue;
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

        robot::common::logger()->warn(
            "{} 开始驱动器边界接管：{:.6f} -> {:.6f} rad，计划位移 {:.6f} rad",
            joint.config.name,
            original_position,
            capture_target,
            planned_distance);

        // 异常处理会重复保持 last_commanded，因此在第一帧前先切换到
        // 已验证的边界内目标，不能回退发送范围外 q0。
        joint.last_commanded = capture_target;
        double previous_measured = original_position;
        double maximum_observed_speed = 0.0;
        auto previous_feedback_time =
            std::chrono::steady_clock::now();
        auto next_cycle = std::chrono::steady_clock::now();
        for (int cycle = 0;
             cycle < kDriverBoundaryCaptureCycles;
             ++cycle)
        {
            if (stop_requested.load())
            {
                throw std::runtime_error(
                    "操作者在肩横滚边界接管阶段中止");
            }
            next_cycle += kControlPeriod;
            joint.motor->commandPositionCsp(capture_target);
            std::this_thread::sleep_until(next_cycle);
            const auto previous_sequence =
                joint.feedback.last_sequence;
            const double measured = updateFeedback(joint);
            if (measured <
                    std::min(original_position, capture_target) -
                        kDriverBoundaryCaptureOvershootRad ||
                measured >
                    std::max(original_position, capture_target) +
                        kDriverBoundaryCaptureOvershootRad)
            {
                throw std::runtime_error(
                    joint.config.name +
                    " 的边界接管反馈越过计划包络");
            }
            if (joint.feedback.last_sequence > previous_sequence)
            {
                const auto feedback_time =
                    std::chrono::steady_clock::now();
                const auto sequence_gap =
                    joint.feedback.last_sequence - previous_sequence;
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
                    measured - previous_measured;
                const double observed_speed =
                    std::abs(position_step) /
                    feedback_interval_seconds;
                maximum_observed_speed = std::max(
                    maximum_observed_speed, observed_speed);
                if (observed_speed >
                    kMaximumDriverBoundaryCaptureSpeedRadPerSecond)
                {
                    std::ostringstream message;
                    message << std::fixed << std::setprecision(6)
                            << joint.config.name
                            << " 的边界接管观测速度="
                            << observed_speed
                            << " rad/s，超过上限 "
                            << kMaximumDriverBoundaryCaptureSpeedRadPerSecond
                            << " rad/s；反馈 " << previous_measured
                            << " -> " << measured
                            << " rad，位移=" << position_step
                            << " rad，实际 dt="
                            << feedback_interval_seconds
                            << " s，sequence_gap=" << sequence_gap;
                    throw std::runtime_error(message.str());
                }
                previous_measured = measured;
                previous_feedback_time = feedback_time;
            }
        }

        const auto final_position = joint.motor->queryPosition();
        const auto status = queryDriverStatus(joint);
        if (!final_position ||
            std::abs(*final_position - capture_target) >
                kFinalToleranceRad ||
            status.mode != 8 || status.fault != 0)
        {
            throw std::runtime_error(
                joint.config.name +
                " 未完成驱动器边界接管，禁止继续回零");
        }

        joint.start_position = capture_target;
        joint.last_commanded = capture_target;
        joint.feedback.measured_position = *final_position;
        robot::common::logger()->info(
            "{} 边界接管完成：mode=8、fault=0，最大观测速度 {:.6f} rad/s",
            joint.config.name,
            maximum_observed_speed);
    }
}

void holdLastCommandsBestEffort(
    std::vector<JointRuntime> &joints)
{
    robot::common::logger()->warn(
        "异常后为避免突然释放，继续发送各关节最后已下达目标 {:.2f} 秒；不发送 STOP，程序退出后仍不得假定电机已释放",
        20.0 * std::chrono::duration<double>(kControlPeriod).count());
    try
    {
        auto next_cycle = std::chrono::steady_clock::now();
        for (int cycle = 0; cycle < 20; ++cycle)
        {
            next_cycle += kControlPeriod;
            sendTargets(joints);
            std::this_thread::sleep_until(next_cycle);
        }
    }
    catch (const std::exception &hold_error)
    {
        robot::common::logger()->error(
            "异常后的保护性 HOLD 发送失败：{}",
            hold_error.what());
    }

    for (auto &joint : joints)
    {
        if (!joint.driver_boundary_capture_target)
        {
            continue;
        }
        try
        {
            const auto final_position = joint.motor->queryPosition();
            const auto status = queryDriverStatus(joint);
            if (final_position)
            {
                robot::common::logger()->warn(
                    "{} 异常后保护性 HOLD 状态：位置 {:.6f} rad、最后目标 {:.6f} rad、mode={}、fault=0x{:08X}",
                    joint.config.name,
                    *final_position,
                    joint.last_commanded,
                    status.mode,
                    static_cast<std::uint32_t>(status.fault));
            }
            else
            {
                robot::common::logger()->error(
                    "{} 异常后保护性 HOLD 的最终位置查询失败；mode={}、fault=0x{:08X}",
                    joint.config.name,
                    status.mode,
                    static_cast<std::uint32_t>(status.fault));
            }
        }
        catch (const std::exception &status_error)
        {
            robot::common::logger()->error(
                "{} 异常后保护性 HOLD 状态复核失败：{}",
                joint.config.name,
                status_error.what());
        }
    }
}

void verifyReadiness(std::vector<JointRuntime> &joints)
{
    robot::common::logger()->info(
        "开始 CSP 就绪探测：仅重复发送各关节当前位置，不产生计划位移");
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 0; cycle < kReadyProbeCycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error("操作者在就绪探测阶段中止");
        }
        next_cycle += kControlPeriod;
        sendTargets(joints);
        std::this_thread::sleep_until(next_cycle);
        for (auto &joint : joints)
        {
            static_cast<void>(updateFeedback(joint));
        }
    }

    for (const auto &joint : joints)
    {
        if (joint.feedback.fresh_cycles < kReadyProbeCycles / 2)
        {
            throw std::runtime_error(
                joint.config.name + " 没有建立稳定 CSP 反馈；未开始后续动作");
        }
    }
    robot::common::logger()->info(
        "{} 个关节均已确认 CSP 就绪", joints.size());
}

void runZeroTrajectory(
    std::vector<JointRuntime> &joints,
    const int move_cycles)
{
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 1; cycle <= move_cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error(
                "操作者中止；程序保持最后一个已发送目标，不发送 STOP");
        }

        const double blend = quinticBlend(
            static_cast<double>(cycle) /
            static_cast<double>(move_cycles));
        for (auto &joint : joints)
        {
            joint.last_commanded = joint.start_position * (1.0 - blend);
        }

        next_cycle += kControlPeriod;
        sendTargets(joints);
        std::this_thread::sleep_until(next_cycle);

        for (auto &joint : joints)
        {
            const double measured = updateFeedback(joint);
            const double lower = std::min(0.0, joint.start_position) -
                                 kAllowedPathOvershootRad;
            const double upper = std::max(0.0, joint.start_position) +
                                 kAllowedPathOvershootRad;
            if (measured < lower || measured > upper)
            {
                throw std::runtime_error(
                    joint.config.name +
                    " 的反馈越过起点到零点轨迹包络，停止推进目标");
            }
        }

        if (cycle % 100 == 0 || cycle == move_cycles)
        {
            robot::common::logger()->info(
                "回零进度：{}/{}（{:.0f}%）",
                cycle,
                move_cycles,
                100.0 * static_cast<double>(cycle) /
                    static_cast<double>(move_cycles));
        }
    }
}

void runTargetSegment(
    std::vector<JointRuntime> &joints,
    const std::vector<double> &targets,
    const std::string &name,
    const double maximum_velocity_rad_per_second)
{
    if (targets.size() != joints.size() ||
        !(maximum_velocity_rad_per_second > 0.0))
    {
        throw std::invalid_argument("无效的分段轨迹参数：" + name);
    }

    std::vector<double> starts;
    starts.reserve(joints.size());
    double maximum_distance = 0.0;
    for (std::size_t index = 0; index < joints.size(); ++index)
    {
        starts.push_back(joints[index].last_commanded);
        maximum_distance = std::max(
            maximum_distance,
            std::abs(targets[index] - starts[index]));
    }
    const double move_seconds = std::max(
        kMinimumMoveSeconds,
        1.875 * maximum_distance / maximum_velocity_rad_per_second);
    const int move_cycles = std::max(
        1,
        static_cast<int>(std::ceil(
            move_seconds /
            std::chrono::duration<double>(kControlPeriod).count())));

    robot::common::logger()->info(
        "分段 {}：预计 {:.2f} 秒，峰值速度不超过 {:.3f} rad/s",
        name,
        move_seconds,
        maximum_velocity_rad_per_second);

    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 1; cycle <= move_cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error(
                "操作者中止；程序保持最后一个已发送目标，不发送 STOP");
        }
        const double blend = quinticBlend(
            static_cast<double>(cycle) /
            static_cast<double>(move_cycles));
        for (std::size_t index = 0; index < joints.size(); ++index)
        {
            joints[index].last_commanded = starts[index] +
                                            (targets[index] - starts[index]) *
                                                blend;
        }

        next_cycle += kControlPeriod;
        sendTargets(joints);
        std::this_thread::sleep_until(next_cycle);

        for (std::size_t index = 0; index < joints.size(); ++index)
        {
            const double measured = updateFeedback(joints[index]);
            const double lower = std::min(starts[index], targets[index]) -
                                 kAllowedPathOvershootRad;
            const double upper = std::max(starts[index], targets[index]) +
                                 kAllowedPathOvershootRad;
            if (measured < lower || measured > upper)
            {
                throw std::runtime_error(
                    joints[index].config.name +
                    " 的反馈越过分段轨迹包络，停止推进目标");
            }
        }

        if (cycle % 100 == 0 || cycle == move_cycles)
        {
            robot::common::logger()->info(
                "{} 进度：{}/{}（{:.0f}%）",
                name,
                cycle,
                move_cycles,
                100.0 * static_cast<double>(cycle) /
                    static_cast<double>(move_cycles));
        }
    }
}

void holdCommandedTargets(
    std::vector<JointRuntime> &joints,
    const int cycles,
    const std::string &name)
{
    robot::common::logger()->info(
        "在 {} 保持 {:.2f} 秒",
        name,
        static_cast<double>(cycles) *
            std::chrono::duration<double>(kControlPeriod).count());
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 0; cycle < cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error(
                "操作者在中间点保持阶段中止；保持最后目标");
        }
        next_cycle += kControlPeriod;
        sendTargets(joints);
        std::this_thread::sleep_until(next_cycle);
        for (auto &joint : joints)
        {
            static_cast<void>(updateFeedback(joint));
        }
    }
}

void holdTargetsAndVerify(std::vector<JointRuntime> &joints)
{
    auto next_cycle = std::chrono::steady_clock::now();
    for (int cycle = 0; cycle < kFinalHoldCycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error(
                "操作者在 HOLD 验证阶段中止；最后目标已经发送");
        }
        next_cycle += kControlPeriod;
        sendTargets(joints);
        std::this_thread::sleep_until(next_cycle);
        for (auto &joint : joints)
        {
            static_cast<void>(updateFeedback(joint));
        }
    }

    std::vector<std::string> failures;
    for (auto &joint : joints)
    {
        const auto position = joint.motor->queryPosition();
        if (!position)
        {
            failures.push_back(joint.config.name + "：最终 0x08 查询失败");
            continue;
        }
        if (std::abs(*position - joint.last_commanded) > kFinalToleranceRad)
        {
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << joint.config.name << "：最终位置="
                    << *position << " rad，目标="
                    << joint.last_commanded << " rad";
            failures.push_back(message.str());
        }
    }
    if (!failures.empty())
    {
        std::ostringstream message;
        message << "以下关节没有进入目标容差 ±"
                << kFinalToleranceRad << " rad：";
        for (const auto &failure : failures)
        {
            message << "\n  " << failure;
        }
        throw std::runtime_error(message.str());
    }
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        const Options options = parseOptions(argc, argv);
        const std::filesystem::path source_dir{TI5_SOURCE_DIR};
        const auto robot_config = robot::ti5::loadRobotConfig(
            source_dir / "config/ti5/t170c/robot.yaml");
        const auto can_config = robot::ti5::loadCanConfig(
            source_dir / "config/ti5/t170c/can.yaml");
        const auto limits = loadSafetyLimits(
            source_dir / "config/ti5/t170c/safety.yaml");
        const auto selected_joints = selectJoints(robot_config);

        printPlan(selected_joints, limits, options);
        if (options.dry_run)
        {
            std::cout
                << "\n无动作检查通过：未打开 CAN，未发送任何帧。\n";
            return 0;
        }

        const MenuAction action = promptMenuAction();
        requireEnvironmentConfirmation();
        requireExclusiveController();

        if (options.bring_up && ::geteuid() != 0)
        {
            throw std::runtime_error(
                "自动拉起 CAN 需要 root；请用文档中的 sudo env 命令运行");
        }
        SingleProcessLock process_lock;

        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        robot::can::CanInterfaceManager interface_manager;
        const auto candidate_interfaces = prepareBodyInterfaces(
            options, can_config, interface_manager);
        const auto mapping = discoverBusInterfaces(
            robot_config.can_buses,
            can_config,
            candidate_interfaces);
        for (const auto &bus_name : kControlledBuses)
        {
            robot::common::logger()->info(
                "逻辑总线映射：{} -> {}",
                bus_name,
                mapping.at(bus_name));
        }

        std::map<std::string, std::unique_ptr<robot::ti5::CanBus>> buses;
        for (const auto &bus_name : kControlledBuses)
        {
            const auto bus_config = std::find_if(
                robot_config.can_buses.begin(),
                robot_config.can_buses.end(),
                [&bus_name](const auto &candidate)
                { return candidate.name == bus_name; });
            if (bus_config == robot_config.can_buses.end())
            {
                throw std::runtime_error(
                    std::string{"缺少逻辑 CAN 总线配置："} + bus_name);
            }
            buses.emplace(
                bus_name,
                std::make_unique<robot::ti5::CanBus>(
                    mapping.at(bus_name),
                    robot::ti5::CanBusOptions{
                        bus_config->expected_node_ids,
                        can_config.receive.use_can_filters,
                        can_config.receive.receive_error_frames,
                        can_config.control.send_failure_threshold}));
        }

        if (action == MenuAction::StopArms)
        {
            const auto arm_configs = selectArmJoints(selected_joints);
            const auto arm_statuses = queryDriverStatuses(
                arm_configs, buses);
            bool all_already_stopped = true;
            bool all_currently_holding = true;
            for (const auto &joint : arm_configs)
            {
                const auto &status = arm_statuses.at(joint.name);
                if (status.fault != 0)
                {
                    std::ostringstream message;
                    message << joint.name << " fault=0x"
                            << std::hex << std::uppercase
                            << static_cast<std::uint32_t>(status.fault)
                            << "，禁止执行自动缓降";
                    throw std::runtime_error(message.str());
                }
                all_already_stopped = all_already_stopped &&
                                      status.mode == 0;
                all_currently_holding = all_currently_holding &&
                                        status.mode == 8;
            }

            if (all_already_stopped)
            {
                requireYes(
                    "双臂 14 轴当前已经全部为 mode=0。程序不会重新建立 CSP，也不会执行缓降，\n"
                    "只会再次发送 0x02 STOP 并核验状态；头部状态不变。");
                sendStopToBothArms(selected_joints, buses, mapping);
                robot::common::logger()->info(
                    "双臂原本已处于 mode=0；仅重复 STOP 并完成状态核验，未执行缓降");
                return 0;
            }

            if (!all_currently_holding)
            {
                throw std::runtime_error(
                    "双臂模式不一致：菜单 2 只允许 14 轴全部 mode=8 后缓降，"
                    "或全部 mode=0 时仅核验 STOP；本次未发送 0x44 或 0x02");
            }

            auto arm_joints = buildCspRuntimes(
                arm_configs,
                limits,
                arm_statuses,
                buses,
                false);
            const auto lowering_plan = buildLoweringPlan(arm_joints);
            printLoweringPlan(arm_joints, lowering_plan);
            requireYes(
                "程序将先在当前位置建立双臂 14 轴 CSP，然后以不超过 0.10 rad/s 的峰值速度\n"
                "依次移动到两个中间点，在第二中间点保持并验证后发送 0x02 STOP。\n"
                "STOP 后仍有约 1/3 的剩余下落行程；请在最后一步前全程可靠托住双臂，\n"
                "清空下方和夹点，并保证物理急停可立即触达。0x02 不是已经证明的硬件失能。");

            try
            {
                verifyReadiness(arm_joints);
                runTargetSegment(
                    arm_joints,
                    lowering_plan.first_intermediate,
                    "缓降中间点 1/2",
                    kLoweringVelocityRadPerSecond);
                holdCommandedTargets(
                    arm_joints,
                    kIntermediateHoldCycles,
                    "缓降中间点 1/2");
                runTargetSegment(
                    arm_joints,
                    lowering_plan.second_intermediate,
                    "缓降中间点 2/2",
                    kLoweringVelocityRadPerSecond);
                holdTargetsAndVerify(arm_joints);
                requireFinalStopConfirmation();
            }
            catch (...)
            {
                // STOP 开始前发生异常时继续保持最后的安全目标，不要突然释放。
                holdLastCommandsBestEffort(arm_joints);
                throw;
            }

            // 放在上述 catch 之外：一旦 STOP 已开始，即使状态核验失败，也不能
            // 再发送 0x44，把部分已经 STOP 的节点重新拉回 CSP。
            sendStopToBothArms(selected_joints, buses, mapping);
            robot::common::logger()->info(
                "PASS：双臂已到达第二中间点并发送 STOP；头部状态未改变");
            robot::common::logger()->warn(
                "最后约 1/3 行程由重力决定；mode=0 不证明真正去使能或释放转矩");
            return 0;
        }

        const auto statuses = queryDriverStatuses(selected_joints, buses);
        auto joints = buildCspRuntimes(
            selected_joints,
            limits,
            statuses,
            buses,
            action == MenuAction::ZeroHome);

        try
        {
            if (action == MenuAction::ZeroHome)
            {
                double maximum_distance = 0.0;
                for (const auto &joint : joints)
                {
                    maximum_distance = std::max(
                        maximum_distance,
                        std::abs(joint.start_position));
                }
                const double move_seconds = std::max(
                    kMinimumMoveSeconds,
                    1.875 * maximum_distance /
                        kMaximumVelocityRadPerSecond);
                const int move_cycles = std::max(
                    1,
                    static_cast<int>(std::ceil(
                        move_seconds /
                        std::chrono::duration<double>(
                            kControlPeriod).count())));

                printCspPreflight(joints, "回零预检通过");
                std::cout
                    << "预计轨迹时间：" << std::fixed
                    << std::setprecision(2) << move_seconds
                    << " 秒；之后保持并验证 1 秒。\n";
                requireYes(
                    "程序将同时把头部和双臂 17 个关节移动到 CAN 电机角 0。\n"
                    "肩横滚受控恢复行程可能明显大于其他轴；这不是碰撞规划器。\n"
                    "请确认当前姿态到零点的路径、夹点和线缆均安全，物理急停可立即触达。");

                captureShouldersAtDriverBoundary(joints);
                verifyReadiness(joints);
                robot::common::logger()->info(
                    "开始同步回零，目标使用 CAN 电机角，不使用 kinematics.yaml 模型偏置");
                runZeroTrajectory(joints, move_cycles);
                for (auto &joint : joints)
                {
                    joint.last_commanded = 0.0;
                }
                holdTargetsAndVerify(joints);
                robot::common::logger()->info(
                    "PASS：头部和双臂 17 个关节均进入电机零点 ±{:.6f} rad 并保持",
                    kFinalToleranceRad);
            }
            else
            {
                printCspPreflight(joints, "当前位置 HOLD 预检通过");
                requireYes(
                    "程序将把刚读取的当前位置作为头部和双臂 17 轴 CSP 目标。\n"
                    "不规划位移，但切换/建立 HOLD 时仍可能出现小幅纠偏；请清空夹点并保持急停可触达。");
                verifyReadiness(joints);
                holdTargetsAndVerify(joints);
                robot::common::logger()->info(
                    "PASS：头部和双臂 17 个关节已在当前位置建立 CSP HOLD");
            }
        }
        catch (...)
        {
            holdLastCommandsBestEffort(joints);
            throw;
        }

        robot::common::logger()->info(
            "未发送 0x01、0x02 STOP、参数写入、Flash 或抱闸控制命令");
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "TI5 独立实机测试失败：{}", error.what());
        return 1;
    }
}
