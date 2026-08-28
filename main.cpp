#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"
#include "ti5/arm/arm.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/controller/arm_controller.hpp"
#include "ti5/controller/hand_controller.hpp"
#include "ti5/controller/head_controller.hpp"
#include "ti5/hand/hand.hpp"
#include "ti5/hand/hand_config.hpp"
#include "ti5/hand/hand_discovery.hpp"
#include "ti5/joint/joint_config_builder.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace
{

using namespace std::chrono_literals;

constexpr double kDefaultArmDeltaRad = 0.01;
constexpr double kDefaultHeadDeltaRad = 0.01;
constexpr std::int32_t kDefaultHandDeltaRaw = 20;
constexpr std::uint8_t kDefaultHandSpeedRaw = 5;
constexpr double kMinimumAngularDeltaRad = 0.001;
constexpr double kMaximumAngularDeltaRad = 0.40;
constexpr std::int32_t kMaximumHandDeltaRaw = 50;
constexpr std::uint8_t kMaximumHandSpeedRaw = 20;
constexpr auto kBodyMinimumMoveDuration = 2s;
constexpr double kBodyMaximumPlannedVelocityRadS = 0.10;
constexpr double kQuinticMaximumSlope = 1.875;
constexpr auto kBodyFinalHoldDuration = 300ms;
constexpr auto kHandMoveDuration = 1s;
constexpr auto kHandHoldDuration = 300ms;
constexpr auto kHandPeriod = 50ms;
constexpr std::uint16_t kHandReachedToleranceRaw = 5;
constexpr auto kFailureHoldDuration = 200ms;

std::atomic<bool> stop_requested{false};

void signalHandler(int)
{
    stop_requested.store(true);
}

struct Options
{
    double arm_delta_rad{kDefaultArmDeltaRad};
    double head_delta_rad{kDefaultHeadDeltaRad};
    std::int32_t hand_delta_raw{kDefaultHandDeltaRaw};
    std::uint8_t hand_speed_raw{kDefaultHandSpeedRaw};
    bool commission_hands{false};
    bool body_only{false};
    bool hold_after_test{false};
    bool dry_run{false};
};

class SingleProcessLock final
{
public:
    SingleProcessLock()
    {
        constexpr const char *path =
            "/tmp/zk_robot_ti5_motion.lock";
        fd_ = ::open(path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
        if (fd_ < 0 && errno == ENOENT)
        {
            fd_ = ::open(path,
                         O_CREAT | O_EXCL | O_RDONLY |
                             O_CLOEXEC | O_NOFOLLOW,
                         0666);
            if (fd_ < 0 && errno == EEXIST)
            {
                fd_ = ::open(
                    path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
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

struct BodyBusBinding
{
    robot::ti5::LogicalCanBus logical_bus;
    std::string interface_name;
};

using BodyBusBindings = std::map<std::string, BodyBusBinding>;

std::filesystem::path configPath(const char *name)
{
#ifdef TI5_SOURCE_DIR
    const std::filesystem::path root{TI5_SOURCE_DIR};
#else
    const std::filesystem::path root{"."};
#endif
    return root / "config" / "ti5" / "t170c" / name;
}

void printUsage(const char *program)
{
    std::cout
        << "用法：\n"
        << "  " << program
        << " [--arm-delta-rad 数值] [--head-delta-rad 数值]"
           " [--hand-delta-raw 整数] [--hand-speed-raw 整数]"
           " [--commission-hands] [--body-only] [--hold-after-test]"
           " [--dry-run]\n\n"
        << "默认流程：左腕、右腕、头部偏航分别小幅移动并返回，"
           "然后左右灵巧手六通道分别小幅移动并返回。\n"
        << "--commission-hands 用于配置尚未开放时的受限首测；"
           "--body-only 跳过灵巧手；--hold-after-test 在本体动作返回后"
           "持续保持当前位置，不自动 STOP；--dry-run 只检查配置，不打开 CAN。\n";
}

double parseDouble(const std::string &text, const std::string &name)
{
    std::size_t used = 0;
    double value = 0.0;
    try
    {
        value = std::stod(text, &used);
    }
    catch (const std::exception &)
    {
        throw std::invalid_argument(name + " 不是有效数值");
    }
    if (used != text.size() || !std::isfinite(value))
    {
        throw std::invalid_argument(name + " 不是有效数值");
    }
    return value;
}

std::int32_t parseInteger(const std::string &text,
                          const std::string &name)
{
    std::size_t used = 0;
    long value = 0;
    try
    {
        value = std::stol(text, &used, 10);
    }
    catch (const std::exception &)
    {
        throw std::invalid_argument(name + " 不是有效整数");
    }
    if (used != text.size() ||
        value < std::numeric_limits<std::int32_t>::min() ||
        value > std::numeric_limits<std::int32_t>::max())
    {
        throw std::invalid_argument(name + " 超出整数范围");
    }
    return static_cast<std::int32_t>(value);
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
        if (argument == "--dry-run")
        {
            options.dry_run = true;
            continue;
        }
        if (argument == "--body-only")
        {
            options.body_only = true;
            continue;
        }
        if (argument == "--commission-hands")
        {
            options.commission_hands = true;
            continue;
        }
        if (argument == "--hold-after-test")
        {
            options.hold_after_test = true;
            continue;
        }
        if (index + 1 >= argc)
        {
            throw std::invalid_argument(argument + " 缺少数值");
        }
        const std::string value{argv[++index]};
        if (argument == "--arm-delta-rad")
        {
            options.arm_delta_rad = parseDouble(value, argument);
        }
        else if (argument == "--head-delta-rad")
        {
            options.head_delta_rad = parseDouble(value, argument);
        }
        else if (argument == "--hand-delta-raw")
        {
            options.hand_delta_raw = parseInteger(value, argument);
        }
        else if (argument == "--hand-speed-raw")
        {
            const auto parsed = parseInteger(value, argument);
            if (parsed < 0 || parsed > 255)
            {
                throw std::invalid_argument(
                    "--hand-speed-raw 必须在 0～255 之间");
            }
            options.hand_speed_raw =
                static_cast<std::uint8_t>(parsed);
        }
        else
        {
            throw std::invalid_argument("未知参数：" + argument);
        }
    }

    const auto valid_angular_delta = [](const double value)
    {
        return value >= kMinimumAngularDeltaRad &&
               value <= kMaximumAngularDeltaRad;
    };
    if (!valid_angular_delta(options.arm_delta_rad) ||
        !valid_angular_delta(options.head_delta_rad))
    {
        throw std::invalid_argument(
            "Arm/Head 测试位移必须在 0.001～0.40 rad 之间");
    }
    const auto hand_delta_magnitude = std::abs(
        static_cast<std::int64_t>(options.hand_delta_raw));
    if (options.hand_delta_raw == 0 ||
        hand_delta_magnitude > kMaximumHandDeltaRaw)
    {
        throw std::invalid_argument(
            "灵巧手原始位置位移必须非零且绝对值不超过 50");
    }
    if (options.hand_speed_raw == 0 ||
        options.hand_speed_raw > kMaximumHandSpeedRaw)
    {
        throw std::invalid_argument(
            "灵巧手原始速度必须在 1～20 之间");
    }
    if (options.body_only && options.commission_hands)
    {
        throw std::invalid_argument(
            "--body-only 与 --commission-hands 不能同时使用");
    }
    return options;
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
        const auto text = entry->path().filename().string();
        if (text.empty() ||
            !std::all_of(text.begin(), text.end(), [](unsigned char value)
                         { return value >= '0' && value <= '9'; }))
        {
            continue;
        }
        long pid = 0;
        try
        {
            pid = std::stol(text);
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
        std::string name;
        if (std::getline(comm, name) && name == expected_name)
        {
            return true;
        }
    }
    return false;
}

void requireExclusiveController()
{
    constexpr std::array<const char *, 6> names{
        "Ti5Control",
        "joint_manager",
        "ti5_follow_arm",
        "ti5_zero_home",
        "ti5_arm_check",
        "robot"};
    for (const char *name : names)
    {
        if (processNamedIsRunning(name))
        {
            throw std::runtime_error(
                std::string{"检测到其他控制程序："} + name);
        }
    }
}

void requireExplicitConfirmation()
{
    const char *value =
        std::getenv("ZK_ROBOT_CONFIRM_ALL_COMPONENT_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "未设置 ZK_ROBOT_CONFIRM_ALL_COMPONENT_TEST=YES；未打开 CAN");
    }
}

void requireHandCommissioningConfirmation()
{
    const char *value =
        std::getenv("ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "灵巧手首测还必须设置 "
            "ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST=YES");
    }
}

std::vector<std::string> prepareAdapterInterfaces(
    const std::string &interface_regex,
    const robot::can::CanAdapterSelector &selector,
    const robot::can::CanInterfaceSettings &settings,
    const bool manage_linux_link)
{
    robot::can::CanInterfaceManager manager;
    const auto all = manager.enumerate(interface_regex);
    const auto selected = manager.selectAdapter(all, selector);
    if (selected.empty())
    {
        throw std::runtime_error("没有找到匹配的 USB-CAN 适配器");
    }

    std::vector<std::string> result;
    result.reserve(selected.size());
    for (const auto &interface : selected)
    {
        const auto ready = manage_linux_link
                               ? manager.prepare(interface.name, settings)
                               : manager.inspect(interface.name);
        if (!ready.up)
        {
            throw std::runtime_error(
                interface.name + " 未处于 UP 状态");
        }
        if (settings.validate_bitrate &&
            (!ready.bitrate || *ready.bitrate != settings.bitrate))
        {
            throw std::runtime_error(
                interface.name + " 波特率不一致");
        }
        result.push_back(ready.name);
    }
    return result;
}

std::vector<std::string> prepareBodyCan(
    const robot::ti5::CanConfig &can_config)
{
    const auto &selector = can_config.socketcan.body_adapter;
    return prepareAdapterInterfaces(
        can_config.socketcan.interface_regex,
        robot::can::CanAdapterSelector{
            selector.selector,
            selector.value,
            selector.expected_channels},
        robot::can::CanInterfaceSettings{
            can_config.socketcan.bitrate,
            static_cast<std::uint32_t>(
                can_config.socketcan.restart_ms.count()),
            can_config.socketcan.reconfigure_wait,
            can_config.socketcan.startup_wait,
            can_config.socketcan.validate_bitrate},
        can_config.socketcan.manage_linux_link);
}

BodyBusBindings discoverBodyBuses(
    const robot::ti5::Ti5RobotConfig &robot_config,
    const robot::ti5::CanConfig &can_config,
    const std::vector<std::string> &candidate_interfaces)
{
    constexpr std::array<const char *, 3> names{
        "left_arm", "right_arm", "head"};
    std::vector<robot::ti5::LogicalCanBus> requested;
    for (const char *name : names)
    {
        const auto found = std::find_if(
            robot_config.can_buses.begin(),
            robot_config.can_buses.end(),
            [name](const auto &bus) { return bus.name == name; });
        if (found == robot_config.can_buses.end())
        {
            throw std::runtime_error(
                std::string{"robot.yaml 缺少逻辑总线："} + name);
        }
        requested.push_back(*found);
    }

    robot::ti5::CanDiscovery discovery;
    const auto result = discovery.discover(
        requested,
        robot::ti5::makeDiscoveryOptions(can_config),
        candidate_interfaces);
    if (!result.success)
    {
        throw std::runtime_error(
            "双臂和头部 CAN 总线发现失败");
    }

    BodyBusBindings bindings;
    for (const auto &logical : result.logical_buses)
    {
        if (!logical.complete || !logical.interface_name)
        {
            throw std::runtime_error(
                logical.bus_name + " CAN 总线不完整");
        }
        const auto configured = std::find_if(
            requested.begin(),
            requested.end(),
            [&logical](const auto &bus)
            { return bus.name == logical.bus_name; });
        if (configured == requested.end())
        {
            throw std::logic_error("发现结果包含未请求的逻辑总线");
        }
        bindings.emplace(
            logical.bus_name,
            BodyBusBinding{*configured, *logical.interface_name});
    }
    if (bindings.size() != names.size())
    {
        throw std::runtime_error("双臂和头部逻辑总线映射不完整");
    }
    return bindings;
}

std::unique_ptr<robot::ti5::CanBus> makeBodyBus(
    const BodyBusBinding &binding,
    const robot::ti5::CanConfig &can_config)
{
    return std::make_unique<robot::ti5::CanBus>(
        binding.interface_name,
        robot::ti5::CanBusOptions{
            binding.logical_bus.expected_node_ids,
            can_config.receive.use_can_filters,
            can_config.receive.receive_error_frames,
            can_config.control.send_failure_threshold});
}

double quinticBlend(const double ratio)
{
    const double t = std::clamp(ratio, 0.0, 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    return 10.0 * t3 - 15.0 * t3 * t + 6.0 * t3 * t2;
}

std::chrono::milliseconds bodyMoveDuration(const double delta_rad)
{
    const double required_seconds =
        kQuinticMaximumSlope * std::abs(delta_rad) /
        kBodyMaximumPlannedVelocityRadS;
    const auto required = std::chrono::duration_cast<
        std::chrono::milliseconds>(
        std::chrono::duration<double>{required_seconds});
    return std::max(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            kBodyMinimumMoveDuration),
        required);
}

template <std::size_t N>
std::array<double, N> interpolate(
    const std::array<double, N> &from,
    const std::array<double, N> &to,
    const double blend)
{
    std::array<double, N> result{};
    for (std::size_t index = 0; index < N; ++index)
    {
        result[index] = from[index] +
                        (to[index] - from[index]) * blend;
    }
    return result;
}

void runBodySegment(
    const char *name,
    robot::ti5::ArmController &left,
    robot::ti5::ArmController &right,
    robot::ti5::HeadController &head,
    const robot::ti5::Arm::JointValues &left_from,
    const robot::ti5::Arm::JointValues &left_to,
    const robot::ti5::Arm::JointValues &right_from,
    const robot::ti5::Arm::JointValues &right_to,
    const robot::ti5::Head::JointValues &head_from,
    const robot::ti5::Head::JointValues &head_to,
    const std::chrono::milliseconds duration,
    const std::chrono::milliseconds period)
{
    const auto cycles = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(duration / period));
    robot::common::logger()->info("整机测试阶段：{}", name);
    auto next = std::chrono::steady_clock::now();
    for (std::size_t cycle = 1; cycle <= cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error("操作者中止整机实机测试");
        }
        const double blend = quinticBlend(
            static_cast<double>(cycle) /
            static_cast<double>(cycles));
        left.setTarget(interpolate(left_from, left_to, blend));
        right.setTarget(interpolate(right_from, right_to, blend));
        head.setTarget(interpolate(head_from, head_to, blend));
        left.update();
        right.update();
        head.update();
        next += period;
        std::this_thread::sleep_until(next);
    }
}

void holdBodyUntilInterrupted(
    robot::ti5::ArmController &left,
    robot::ti5::ArmController &right,
    robot::ti5::HeadController &head,
    const std::chrono::milliseconds period)
{
    robot::common::logger()->info(
        "双臂和头部已经返回起点，开始持续刷新当前位置目标；"
        "按 Ctrl+C 结束刷新，程序不会发送 STOP");
    auto next = std::chrono::steady_clock::now();
    while (!stop_requested.load())
    {
        left.update();
        right.update();
        head.update();
        next += period;
        std::this_thread::sleep_until(next);
    }
    robot::common::logger()->warn(
        "收到结束信号，已停止周期刷新；未发送 STOP，驱动器保留最后目标和"
        "当前运行模式，但程序退出后不得假定仍在主动刷新保持");
}

void runTaskWhileHoldingBody(
    const std::function<void()> &task,
    robot::ti5::ArmController &left,
    robot::ti5::ArmController &right,
    robot::ti5::HeadController &head,
    const std::chrono::milliseconds period)
{
    if (!task)
    {
        return;
    }

    robot::common::logger()->info(
        "本体已经返回起点；执行灵巧手动作期间继续刷新双臂和头部保持目标");
    auto task_future = std::async(std::launch::async, task);
    try
    {
        auto next = std::chrono::steady_clock::now();
        while (task_future.wait_for(0ms) != std::future_status::ready)
        {
            left.update();
            right.update();
            head.update();
            next += period;
            std::this_thread::sleep_until(next);
        }
        task_future.get();
    }
    catch (...)
    {
        const auto original_error = std::current_exception();
        stop_requested.store(true);
        if (task_future.valid())
        {
            try
            {
                task_future.get();
            }
            catch (const std::exception &)
            {
            }
        }
        std::rethrow_exception(original_error);
    }
}

void protectiveBodyHold(
    robot::ti5::ArmController &left,
    robot::ti5::ArmController &right,
    robot::ti5::HeadController &head,
    const std::chrono::milliseconds period) noexcept
{
    robot::common::logger()->warn(
        "异常后继续刷新仍处于 Running 的最后目标 0.20 秒；"
        "不自动发送 STOP");
    const auto deadline =
        std::chrono::steady_clock::now() + kFailureHoldDuration;
    while (std::chrono::steady_clock::now() < deadline)
    {
        const auto update = [](auto &controller)
        {
            using Controller = std::decay_t<decltype(controller)>;
            if (controller.state() == Controller::ControlState::Running)
            {
                try
                {
                    controller.update();
                }
                catch (const std::exception &)
                {
                }
            }
        };
        update(left);
        update(right);
        update(head);
        std::this_thread::sleep_for(period);
    }
}

void requireInteractiveConfirmation(const Options &options)
{
    std::cout
        << "\n即将执行以下实机动作：\n"
        << "  left_wrist_roll  +" << options.arm_delta_rad
        << " rad 后返回\n"
        << "  right_wrist_roll -" << options.arm_delta_rad
        << " rad 后返回\n"
        << "  neck_yaw         +" << options.head_delta_rad
        << " rad 后返回\n"
        << "  双臂单程时长约 "
        << static_cast<double>(
               bodyMoveDuration(options.arm_delta_rad).count()) /
               1000.0
        << " s，头部单程时长约 "
        << static_cast<double>(
               bodyMoveDuration(options.head_delta_rad).count()) /
               1000.0
        << " s，规划峰值速度不超过 "
        << kBodyMaximumPlannedVelocityRadS << " rad/s\n";
    if (!options.body_only)
    {
        std::cout
            << "  左右灵巧手六通道分别移动 "
            << options.hand_delta_raw
            << " 个原始位置单位后返回，原始速度 "
            << static_cast<unsigned int>(options.hand_speed_raw)
            << "\n";
    }
    std::cout << "双臂必须全程有人可靠支撑；";
    if (options.hold_after_test)
    {
        if (options.body_only)
        {
            std::cout
                << "本体返回起点后会持续刷新当前位置目标，不自动发送"
                   " STOP；按 Ctrl+C 结束保持。\n";
        }
        else
        {
            std::cout
                << "本体返回起点后会持续刷新当前位置目标；灵巧手动作完成"
                   "后仍不自动发送 STOP，按 Ctrl+C 结束保持。\n";
        }
    }
    else
    {
        std::cout
            << "成功返回起点后程序会请求并确认 Arm/Head 的 mode=0。\n";
    }
    std::cout << "确认请输入 y 或 Y；其他输入取消：";
    std::string confirmation;
    std::getline(std::cin, confirmation);
    if (confirmation != "y" && confirmation != "Y")
    {
        throw std::runtime_error(
            "操作者取消；未发送位置命令或 STOP");
    }
}

void printArmState(const robot::ti5::Arm &arm,
                   const robot::ti5::ArmState &state)
{
    for (std::size_t index = 0; index < robot::ti5::Arm::kJointCount;
         ++index)
    {
        const auto &joint = state.joints[index];
        std::cout << "  " << arm.jointNames()[index] << " position="
                  << (joint.position_rad
                          ? std::to_string(*joint.position_rad)
                          : "unknown")
                  << " mode="
                  << (joint.run_mode
                          ? std::to_string(*joint.run_mode)
                          : "unknown")
                  << " fault="
                  << (joint.fault_bits
                          ? std::to_string(*joint.fault_bits)
                          : "unknown")
                  << '\n';
    }
}

void printHeadState(const robot::ti5::Head &head,
                    const robot::ti5::HeadState &state)
{
    for (std::size_t index = 0; index < robot::ti5::Head::kJointCount;
         ++index)
    {
        const auto &joint = state.joints[index];
        std::cout << "  " << head.jointNames()[index] << " position="
                  << (joint.position_rad
                          ? std::to_string(*joint.position_rad)
                          : "unknown")
                  << " mode="
                  << (joint.run_mode
                          ? std::to_string(*joint.run_mode)
                          : "unknown")
                  << " fault="
                  << (joint.fault_bits
                          ? std::to_string(*joint.fault_bits)
                          : "unknown")
                  << '\n';
    }
}

void runBodyTest(
    const Options &options,
    const robot::ti5::Ti5RobotConfig &robot_config,
    const robot::ti5::CanConfig &can_config,
    const std::vector<robot::ti5::JointConfig> &left_configs,
    const std::vector<robot::ti5::JointConfig> &right_configs,
    const std::vector<robot::ti5::JointConfig> &head_configs,
    const std::function<void()> &after_body_actions)
{
    const auto candidates = prepareBodyCan(can_config);
    const auto bindings = discoverBodyBuses(
        robot_config, can_config, candidates);
    for (const auto &[name, binding] : bindings)
    {
        robot::common::logger()->info(
            "本体总线映射：{} -> {}", name, binding.interface_name);
    }

    const auto period = std::chrono::milliseconds{
        static_cast<std::chrono::milliseconds::rep>(
            1000 / can_config.control.frequency_hz)};
    robot::ti5::ArmOptions arm_options;
    arm_options.control_period = period;
    arm_options.inter_frame_gap =
        can_config.control.inter_frame_gap;
    arm_options.maximum_stale_cycles =
        can_config.watchdog.stale_feedback_cycles;
    robot::ti5::HeadOptions head_options;
    head_options.control_period = period;
    head_options.inter_frame_gap =
        can_config.control.inter_frame_gap;
    head_options.maximum_stale_cycles =
        can_config.watchdog.stale_feedback_cycles;

    robot::ti5::Arm left_arm(
        robot::ti5::ArmSide::Left,
        makeBodyBus(bindings.at("left_arm"), can_config),
        left_configs,
        arm_options);
    robot::ti5::Arm right_arm(
        robot::ti5::ArmSide::Right,
        makeBodyBus(bindings.at("right_arm"), can_config),
        right_configs,
        arm_options);
    robot::ti5::Head head(
        makeBodyBus(bindings.at("head"), can_config),
        head_configs,
        head_options);

    left_arm.prepare();
    right_arm.prepare();
    head.prepare();
    const auto left_state = left_arm.readState();
    const auto right_state = right_arm.readState();
    const auto head_state = head.readState();
    std::cout << "\n左臂当前位置：\n";
    printArmState(left_arm, left_state);
    std::cout << "右臂当前位置：\n";
    printArmState(right_arm, right_state);
    std::cout << "头部当前位置：\n";
    printHeadState(head, head_state);

    robot::ti5::Arm::JointValues left_start{};
    robot::ti5::Arm::JointValues right_start{};
    robot::ti5::Head::JointValues head_start{};
    for (std::size_t index = 0; index < left_start.size(); ++index)
    {
        if (!left_state.joints[index].position_rad ||
            !right_state.joints[index].position_rad)
        {
            throw std::runtime_error("双臂缺少完整位置反馈");
        }
        left_start[index] = *left_state.joints[index].position_rad;
        right_start[index] = *right_state.joints[index].position_rad;
    }
    for (std::size_t index = 0; index < head_start.size(); ++index)
    {
        if (!head_state.joints[index].position_rad)
        {
            throw std::runtime_error("头部缺少完整位置反馈");
        }
        head_start[index] = *head_state.joints[index].position_rad;
    }

    left_arm.validatePositions(left_start);
    right_arm.validatePositions(right_start);
    head.validatePositions(head_start);
    auto left_target = left_start;
    auto right_target = right_start;
    auto head_target = head_start;
    left_target[6] += options.arm_delta_rad;
    right_target[6] -= options.arm_delta_rad;
    head_target[0] += options.head_delta_rad;
    left_arm.validatePositions(left_target);
    right_arm.validatePositions(right_target);
    head.validatePositions(head_target);

    requireInteractiveConfirmation(options);
    robot::ti5::ArmController left_controller(left_arm);
    robot::ti5::ArmController right_controller(right_arm);
    robot::ti5::HeadController head_controller(head);
    try
    {
        // 三个部件位于独立 CAN 总线。并行建立当前位置控制，避免顺序启动
        // 期间最先完成的部件反馈已经超过 maximum_feedback_age。
        auto left_start_future = std::async(
            std::launch::async,
            [&]()
            {
                left_arm.startPositionControlAtCurrentPosition();
                left_controller.start();
            });
        auto right_start_future = std::async(
            std::launch::async,
            [&]()
            {
                right_arm.startPositionControlAtCurrentPosition();
                right_controller.start();
            });
        auto head_start_future = std::async(
            std::launch::async,
            [&]()
            {
                head.startPositionControlAtCurrentPosition();
                head_controller.start();
            });
        left_start_future.get();
        right_start_future.get();
        head_start_future.get();

        const auto arm_move_duration =
            bodyMoveDuration(options.arm_delta_rad);
        const auto head_move_duration =
            bodyMoveDuration(options.head_delta_rad);

        runBodySegment(
            "左腕移出", left_controller, right_controller, head_controller,
            left_start, left_target,
            right_start, right_start,
            head_start, head_start,
            arm_move_duration, period);
        runBodySegment(
            "左腕返回", left_controller, right_controller, head_controller,
            left_target, left_start,
            right_start, right_start,
            head_start, head_start,
            arm_move_duration, period);
        runBodySegment(
            "右腕移出", left_controller, right_controller, head_controller,
            left_start, left_start,
            right_start, right_target,
            head_start, head_start,
            arm_move_duration, period);
        runBodySegment(
            "右腕返回", left_controller, right_controller, head_controller,
            left_start, left_start,
            right_target, right_start,
            head_start, head_start,
            arm_move_duration, period);
        runBodySegment(
            "头部偏航移出", left_controller, right_controller, head_controller,
            left_start, left_start,
            right_start, right_start,
            head_start, head_target,
            head_move_duration, period);
        runBodySegment(
            "头部偏航返回", left_controller, right_controller, head_controller,
            left_start, left_start,
            right_start, right_start,
            head_target, head_start,
            head_move_duration, period);
        runBodySegment(
            "返回点保持", left_controller, right_controller, head_controller,
            left_start, left_start,
            right_start, right_start,
            head_start, head_start,
            kBodyFinalHoldDuration, period);

        if (!left_controller.targetReached(0.003) ||
            !right_controller.targetReached(0.003) ||
            !head_controller.targetReached(0.003))
        {
            throw std::runtime_error(
                "双臂或头部没有回到起始位置容差内");
        }

        runTaskWhileHoldingBody(
            after_body_actions,
            left_controller,
            right_controller,
            head_controller,
            period);

        if (options.hold_after_test)
        {
            holdBodyUntilInterrupted(
                left_controller,
                right_controller,
                head_controller,
                period);
            return;
        }

        head_controller.stopAndConfirm();
        right_controller.stopAndConfirm();
        left_controller.stopAndConfirm();
        robot::common::logger()->info(
            "双臂和头部移动、返回及 STOP 确认通过");
    }
    catch (...)
    {
        protectiveBodyHold(
            left_controller,
            right_controller,
            head_controller,
            period);
        throw;
    }
}

std::vector<std::string> prepareHandCan(
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::hand::HandConfig &hand_config)
{
    const auto &transport = hand_config.transport;
    return prepareAdapterInterfaces(
        can_config.socketcan.interface_regex,
        transport.adapter_selector,
        robot::can::CanInterfaceSettings{
            transport.bitrate,
            transport.restart_ms,
            transport.reconfigure_wait,
            transport.startup_wait,
            transport.validate_bitrate},
        transport.manage_linux_link);
}

robot::ti5::Hand::PositionValues makeHandTarget(
    const robot::ti5::Hand::PositionValues &start,
    const std::int32_t delta)
{
    robot::ti5::Hand::PositionValues target{};
    for (std::size_t index = 0; index < target.size(); ++index)
    {
        const auto value = static_cast<std::int64_t>(start[index]) + delta;
        if (value < 0 ||
            value > std::numeric_limits<std::uint16_t>::max())
        {
            throw std::out_of_range(
                "灵巧手测试目标超出 uint16 原始位置范围");
        }
        target[index] = static_cast<std::uint16_t>(value);
    }
    return target;
}

robot::ti5::Hand::PositionValues interpolateHand(
    const robot::ti5::Hand::PositionValues &from,
    const robot::ti5::Hand::PositionValues &to,
    const double blend)
{
    robot::ti5::Hand::PositionValues result{};
    for (std::size_t index = 0; index < result.size(); ++index)
    {
        const double value = static_cast<double>(from[index]) +
                             (static_cast<double>(to[index]) -
                              static_cast<double>(from[index])) *
                                 blend;
        result[index] = static_cast<std::uint16_t>(std::llround(value));
    }
    return result;
}

void runHandSegment(
    const char *name,
    robot::ti5::HandController &controller,
    const robot::ti5::Hand::PositionValues &from,
    const robot::ti5::Hand::PositionValues &to,
    const robot::ti5::Hand::SpeedValues &speeds,
    const std::chrono::milliseconds duration)
{
    const auto cycles = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(duration / kHandPeriod));
    robot::common::logger()->info("灵巧手测试阶段：{}", name);
    auto next = std::chrono::steady_clock::now();
    for (std::size_t cycle = 1; cycle <= cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error("操作者中止灵巧手实机测试");
        }
        const double blend = quinticBlend(
            static_cast<double>(cycle) /
            static_cast<double>(cycles));
        controller.setTarget(
            interpolateHand(from, to, blend), speeds);
        controller.update();
        next += kHandPeriod;
        std::this_thread::sleep_until(next);
    }
}

void runOneHandTest(
    const char *name,
    robot::ti5::HandController &controller,
    const Options &options)
{
    robot::ti5::Hand::SpeedValues speeds{};
    speeds.fill(options.hand_speed_raw);
    controller.start(speeds);
    const auto start = controller.targetPositions();
    const auto target = makeHandTarget(start, options.hand_delta_raw);
    runHandSegment(name, controller, start, target, speeds, kHandMoveDuration);
    runHandSegment("端点保持", controller, target, target, speeds,
                   kHandHoldDuration);
    if (!controller.targetReachedRaw(kHandReachedToleranceRaw))
    {
        throw std::runtime_error(
            std::string{name} + " 没有到达测试目标容差");
    }
    runHandSegment("返回", controller, target, start, speeds,
                   kHandMoveDuration);
    runHandSegment("返回点保持", controller, start, start, speeds,
                   kHandHoldDuration);
    if (!controller.targetReachedRaw(kHandReachedToleranceRaw))
    {
        throw std::runtime_error(
            std::string{name} + " 没有返回起始位置容差");
    }
    controller.pause();
}

struct HandBusMapping
{
    std::string left_interface;
    std::string right_interface;
};

HandBusMapping prepareAndDiscoverHandBuses(
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::hand::HandConfig &hand_config)
{
    const auto candidates = prepareHandCan(can_config, hand_config);
    robot::ti5::hand::HandDiscovery discovery;
    const auto result = discovery.discover(hand_config, candidates);
    if (!result.success ||
        !result.left_interface || !result.right_interface)
    {
        const std::string detail = result.errors.empty()
                                       ? std::string{}
                                       : "：" + result.errors.front();
        throw std::runtime_error(
            "左右灵巧手 CAN 接口发现失败" + detail);
    }
    if (*result.left_interface == *result.right_interface)
    {
        throw std::runtime_error(
            "左右手被发现于同一 CAN 接口；当前独立接收结构拒绝并发打开");
    }

    HandBusMapping mapping{
        *result.left_interface,
        *result.right_interface};
    robot::common::logger()->info(
        "灵巧手总线映射：left_hand -> {}, right_hand -> {}",
        mapping.left_interface,
        mapping.right_interface);
    return mapping;
}

void runHandTest(
    const Options &options,
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::hand::HandConfig &hand_config)
{
    const auto mapping =
        prepareAndDiscoverHandBuses(can_config, hand_config);

    robot::ti5::Hand left_hand(
        robot::ti5::HandSide::Left,
        hand_config.left,
        mapping.left_interface,
        hand_config.discovery.response_timeout);
    robot::ti5::Hand right_hand(
        robot::ti5::HandSide::Right,
        hand_config.right,
        mapping.right_interface,
        hand_config.discovery.response_timeout);
    robot::ti5::HandController left_controller(left_hand);
    robot::ti5::HandController right_controller(right_hand);
    try
    {
        runOneHandTest("左手六通道移出", left_controller, options);
        runOneHandTest("右手六通道移出", right_controller, options);
        robot::common::logger()->info(
            "左右灵巧手移动和返回通过；pause 仅停止继续发送命令");
    }
    catch (...)
    {
        robot::common::logger()->warn(
            "灵巧手异常后不继续发送命令；协议没有已确认的 STOP，"
            "不能假定灵巧手已释放或停止");
        throw;
    }
}

bool handControlConfigured(
    const robot::ti5::hand::HandConfig &config)
{
    const auto ready = [](const auto &side)
    {
        return side.discovery_enabled &&
               side.protocol_verified &&
               side.control_enabled;
    };
    return ready(config.left) && ready(config.right);
}

void enableHandCommissioning(
    robot::ti5::hand::HandConfig &config)
{
    if (!config.left.discovery_enabled ||
        !config.right.discovery_enabled)
    {
        throw std::runtime_error(
            "灵巧手首测要求左右手 discovery.enabled 均为 true");
    }
    // 只修改 main 当前进程中的配置副本，不写回 hands.yaml，也不改变
    // Hand 的正常控制门槛。
    config.left.protocol_verified = true;
    config.left.control_enabled = true;
    config.right.protocol_verified = true;
    config.right.control_enabled = true;
}

} // namespace

int main(const int argc, char **argv)
{
    try
    {
        const auto options = parseOptions(argc, argv);
        const auto robot_config = robot::ti5::loadRobotConfig(
            configPath("robot.yaml"));
        const auto can_config = robot::ti5::loadCanConfig(
            configPath("can.yaml"));
        const auto safety = robot::ti5::loadJointSafetyConfig(
            configPath("safety.yaml"));
        const auto kinematics = robot::ti5::loadKinematicsConfig(
            configPath("kinematics.yaml"));
        auto hand_config = robot::ti5::hand::loadHandConfig(
            configPath("hands.yaml"));

        const auto left_model =
            kinematics.models.find("t7_t170_left_arm");
        const auto right_model =
            kinematics.models.find("t7_t170_right_arm");
        if (left_model == kinematics.models.end() ||
            right_model == kinematics.models.end())
        {
            throw std::runtime_error(
                "kinematics.yaml 缺少左右臂模型");
        }
        const auto left_configs = robot::ti5::makeJointConfigs(
            robot_config, safety, &left_model->second);
        const auto right_configs = robot::ti5::makeJointConfigs(
            robot_config, safety, &right_model->second);
        const auto head_configs = robot::ti5::makeJointConfigs(
            robot_config, safety);

        std::cout
            << "TI5 整机实机测试计划：双臂腕部、头部偏航"
            << (options.body_only ? "。" : "、左右灵巧手。")
            << "\n"
            << "Arm 位移=" << options.arm_delta_rad
            << " rad，Head 位移=" << options.head_delta_rad
            << " rad，Hand 原始位移=" << options.hand_delta_raw
            << "，Hand 原始速度="
            << static_cast<unsigned int>(options.hand_speed_raw)
            << "。\n";
        if (options.hold_after_test)
        {
            std::cout
                << "保持模式：本体返回起点后持续刷新当前位置目标";
            if (!options.body_only)
            {
                std::cout << "，并在保持期间执行灵巧手动作";
            }
            std::cout << "；全部动作结束后仍不自动发送 STOP。\n";
        }
        if (!options.body_only && !handControlConfigured(hand_config))
        {
            std::cout
                << "注意：hands.yaml 尚未同时开放左右手的"
                   " protocol_verified 和 control.enabled。\n";
            if (options.commission_hands)
            {
                std::cout
                    << "已选择灵巧手首测模式；仅在本次进程内临时开放"
                       "受限测试，不修改配置文件。\n";
            }
        }
        if (options.dry_run)
        {
            std::cout << "仅核对配置完成；未打开 CAN。\n";
            return 0;
        }
        if (!options.body_only && !handControlConfigured(hand_config))
        {
            if (!options.commission_hands)
            {
                throw std::runtime_error(
                    "左右手控制配置尚未开放；可先使用 --body-only，"
                    "或明确使用 --commission-hands 做受限首测");
            }
            requireHandCommissioningConfirmation();
            enableHandCommissioning(hand_config);
        }

        requireExplicitConfirmation();
        if (::geteuid() == 0)
        {
            throw std::runtime_error(
                "请以 kuang 用户运行，不要使用 sudo/root");
        }
        SingleProcessLock process_lock;
        requireExclusiveController();
        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        std::function<void()> hand_actions;
        if (!options.body_only)
        {
            robot::common::logger()->info(
                "本体运动前先检查左右灵巧手接口和状态反馈");
            static_cast<void>(
                prepareAndDiscoverHandBuses(can_config, hand_config));
            robot::common::logger()->info(
                "左右灵巧手预检查通过；尚未发送位置命令");
            hand_actions = [&]()
            {
                runHandTest(options, can_config, hand_config);
            };
        }

        runBodyTest(
            options,
            robot_config,
            can_config,
            left_configs,
            right_configs,
            head_configs,
            hand_actions);
        if (options.hold_after_test)
        {
            robot::common::logger()->info(
                "TI5 本体持续保持流程已结束；未发送 STOP");
        }
        else
        {
            robot::common::logger()->info(
                "TI5 整机实机测试全部完成");
        }
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "TI5 整机实机测试失败：{}", error.what());
        return 1;
    }
}
