#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/controller/hand_controller.hpp"
#include "ti5/hand/hand.hpp"
#include "ti5/hand/hand_config.hpp"
#include "ti5/hand/hand_discovery.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;

constexpr std::int32_t kDefaultDeltaRaw = 20;
constexpr std::uint8_t kDefaultSpeedRaw = 5;
constexpr std::int32_t kMaximumDeltaRaw = 50;
constexpr std::uint8_t kMaximumSpeedRaw = 20;
constexpr auto kMoveDuration = 1s;
constexpr auto kEndpointHoldDuration = 300ms;
constexpr auto kControlPeriod = 50ms;
constexpr std::uint16_t kReachedToleranceRaw = 5;

std::atomic<bool> stop_requested{false};

void signalHandler(int)
{
    stop_requested.store(true);
}

enum class SideSelection
{
    Left,
    Right,
    Both
};

struct Options
{
    SideSelection side{SideSelection::Both};
    bool side_set{false};
    std::optional<std::size_t> channel;
    std::int32_t delta_raw{kDefaultDeltaRaw};
    std::uint8_t speed_raw{kDefaultSpeedRaw};
    bool read_only{false};
    bool commission{false};
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
        << " --side left|right|both [--channel all|0..5]"
           " [--delta-raw 整数] [--speed-raw 整数]"
           " [--read-only] [--commission] [--dry-run]\n\n"
        << "示例：\n"
        << "  " << program << " --side left --read-only\n"
        << "  " << program
        << " --side right --channel 2 --delta-raw 20"
           " --speed-raw 5 --commission\n\n"
        << "--read-only 只发现接口并读取状态，不发送位置命令。\n"
        << "--commission 仅在 hands.yaml 尚未开放控制时，"
           "临时开放本次进程；不会修改配置文件。\n"
        << "通道 0～5 对应协议中的六个原始位置通道；"
           "all 表示六通道一起运动。\n"
        << "--dry-run 只核对参数和配置，不打开 CAN。\n";
}

long parseInteger(const std::string &text, const std::string &name)
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
    if (used != text.size())
    {
        throw std::invalid_argument(name + " 不是有效整数");
    }
    return value;
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
        if (argument == "--read-only")
        {
            options.read_only = true;
            continue;
        }
        if (argument == "--commission")
        {
            options.commission = true;
            continue;
        }
        if (argument == "--dry-run")
        {
            options.dry_run = true;
            continue;
        }
        if (index + 1 >= argc)
        {
            throw std::invalid_argument(argument + " 缺少数值");
        }
        const std::string value{argv[++index]};
        if (argument == "--side")
        {
            if (value == "left")
            {
                options.side = SideSelection::Left;
            }
            else if (value == "right")
            {
                options.side = SideSelection::Right;
            }
            else if (value == "both")
            {
                options.side = SideSelection::Both;
            }
            else
            {
                throw std::invalid_argument(
                    "--side 只能是 left、right 或 both");
            }
            options.side_set = true;
        }
        else if (argument == "--channel")
        {
            if (value == "all")
            {
                options.channel.reset();
            }
            else
            {
                const auto parsed = parseInteger(value, argument);
                if (parsed < 0 ||
                    parsed >= static_cast<long>(
                        robot::ti5::hand::kAoyiChannelCount))
                {
                    throw std::invalid_argument(
                        "--channel 只能是 all 或 0～5");
                }
                options.channel = static_cast<std::size_t>(parsed);
            }
        }
        else if (argument == "--delta-raw")
        {
            const auto parsed = parseInteger(value, argument);
            if (parsed < std::numeric_limits<std::int32_t>::min() ||
                parsed > std::numeric_limits<std::int32_t>::max())
            {
                throw std::invalid_argument(
                    "--delta-raw 超出整数范围");
            }
            options.delta_raw = static_cast<std::int32_t>(parsed);
        }
        else if (argument == "--speed-raw")
        {
            const auto parsed = parseInteger(value, argument);
            if (parsed < 0 || parsed > 255)
            {
                throw std::invalid_argument(
                    "--speed-raw 必须在 0～255 之间");
            }
            options.speed_raw = static_cast<std::uint8_t>(parsed);
        }
        else
        {
            throw std::invalid_argument("未知参数：" + argument);
        }
    }

    if (!options.side_set)
    {
        throw std::invalid_argument(
            "实机检查必须显式提供 --side left|right|both");
    }
    const auto delta_magnitude = std::abs(
        static_cast<std::int64_t>(options.delta_raw));
    if (options.delta_raw == 0 || delta_magnitude > kMaximumDeltaRaw)
    {
        throw std::invalid_argument(
            "--delta-raw 必须非零且绝对值不超过 50");
    }
    if (options.speed_raw == 0 || options.speed_raw > kMaximumSpeedRaw)
    {
        throw std::invalid_argument(
            "--speed-raw 必须在 1～20 之间");
    }
    if (options.read_only && options.commission)
    {
        throw std::invalid_argument(
            "只读检查不需要 --commission");
    }
    return options;
}

bool includesLeft(const SideSelection side)
{
    return side == SideSelection::Left || side == SideSelection::Both;
}

bool includesRight(const SideSelection side)
{
    return side == SideSelection::Right || side == SideSelection::Both;
}

std::string selectionName(const SideSelection side)
{
    if (side == SideSelection::Left)
    {
        return "左手";
    }
    if (side == SideSelection::Right)
    {
        return "右手";
    }
    return "左右手";
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

void requireEnvironmentConfirmation()
{
    const char *value = std::getenv("ZK_ROBOT_CONFIRM_HAND_CHECK");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "未设置 ZK_ROBOT_CONFIRM_HAND_CHECK=YES；未打开 CAN");
    }
}

void requireCommissionConfirmation()
{
    const char *value =
        std::getenv("ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "未验证的灵巧手运动还必须设置 "
            "ZK_ROBOT_CONFIRM_UNVERIFIED_HAND_TEST=YES");
    }
}

bool controlConfigured(const robot::ti5::hand::HandSideConfig &config)
{
    return config.discovery_enabled &&
           config.protocol_verified &&
           config.control_enabled;
}

void configureSelectedSides(robot::ti5::hand::HandConfig &config,
                            const Options &options)
{
    if (includesLeft(options.side) && !config.left.discovery_enabled)
    {
        throw std::runtime_error(
            "hands.yaml 未开放左手接口发现");
    }
    if (includesRight(options.side) && !config.right.discovery_enabled)
    {
        throw std::runtime_error(
            "hands.yaml 未开放右手接口发现");
    }
    if (!includesLeft(options.side))
    {
        config.left.discovery_enabled = false;
    }
    if (!includesRight(options.side))
    {
        config.right.discovery_enabled = false;
    }

    const auto prepare = [&options](
                             robot::ti5::hand::HandSideConfig &side)
    {
        if (!side.discovery_enabled || options.read_only ||
            controlConfigured(side))
        {
            return;
        }
        if (!options.commission)
        {
            throw std::runtime_error(
                side.name +
                " 控制配置尚未开放；请先使用 --read-only，"
                "或明确添加 --commission 做受限运动测试");
        }
        side.protocol_verified = true;
        side.control_enabled = true;
    };
    prepare(config.left);
    prepare(config.right);
}

std::vector<std::string> prepareHandCan(
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::hand::HandConfig &hand_config)
{
    robot::can::CanInterfaceManager manager;
    const auto all = manager.enumerate(
        can_config.socketcan.interface_regex);
    const auto selected = manager.selectAdapter(
        all, hand_config.transport.adapter_selector);

    const robot::can::CanInterfaceSettings settings{
        hand_config.transport.bitrate,
        hand_config.transport.restart_ms,
        hand_config.transport.reconfigure_wait,
        hand_config.transport.startup_wait,
        hand_config.transport.validate_bitrate};

    std::vector<std::string> result;
    result.reserve(selected.size());
    for (const auto &interface : selected)
    {
        const auto ready = hand_config.transport.manage_linux_link
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

struct HandBusMapping
{
    std::optional<std::string> left;
    std::optional<std::string> right;
};

HandBusMapping discoverHandBuses(
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::hand::HandConfig &hand_config)
{
    const auto candidates = prepareHandCan(can_config, hand_config);
    robot::ti5::hand::HandDiscovery discovery;
    const auto result = discovery.discover(hand_config, candidates);
    if (!result.success)
    {
        const std::string detail = result.errors.empty()
                                       ? std::string{}
                                       : "：" + result.errors.front();
        throw std::runtime_error(
            "灵巧手 CAN 接口发现失败" + detail);
    }
    if (hand_config.left.discovery_enabled && !result.left_interface)
    {
        throw std::runtime_error("没有找到左手 CAN 接口");
    }
    if (hand_config.right.discovery_enabled && !result.right_interface)
    {
        throw std::runtime_error("没有找到右手 CAN 接口");
    }

    if (result.left_interface)
    {
        robot::common::logger()->info(
            "灵巧手总线映射：left_hand -> {}",
            *result.left_interface);
    }
    if (result.right_interface)
    {
        robot::common::logger()->info(
            "灵巧手总线映射：right_hand -> {}",
            *result.right_interface);
    }
    return {result.left_interface, result.right_interface};
}

void printState(const std::string &name,
                const robot::ti5::HandState &state)
{
    std::cout << name << "当前位置原始值：";
    for (const auto value : state.positions_raw)
    {
        std::cout << ' ' << value;
    }
    std::cout << '\n';
    if (state.forces_raw)
    {
        std::cout << name << "力反馈原始值：";
        for (const auto value : *state.forces_raw)
        {
            std::cout << ' ' << value;
        }
        std::cout << '\n';
    }
}

void readOneHand(const robot::ti5::HandSide side,
                 const robot::ti5::hand::HandSideConfig &config,
                 const std::string &interface_name,
                 const std::chrono::milliseconds timeout)
{
    robot::ti5::Hand hand(side, config, interface_name, timeout);
    const auto state = hand.readState();
    if (!state)
    {
        throw std::runtime_error(config.name + " 没有返回状态");
    }
    printState(config.name, *state);
}

robot::ti5::Hand::PositionValues makeTarget(
    const robot::ti5::Hand::PositionValues &start,
    const Options &options)
{
    auto target = start;
    for (std::size_t index = 0; index < target.size(); ++index)
    {
        if (options.channel && *options.channel != index)
        {
            continue;
        }
        const auto value = static_cast<std::int64_t>(start[index]) +
                           options.delta_raw;
        if (value < 0 ||
            value > std::numeric_limits<std::uint16_t>::max())
        {
            throw std::out_of_range(
                "灵巧手测试目标超出原始位置范围；可反向测试");
        }
        target[index] = static_cast<std::uint16_t>(value);
    }
    return target;
}

robot::ti5::Hand::PositionValues interpolate(
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
        result[index] =
            static_cast<std::uint16_t>(std::llround(value));
    }
    return result;
}

double quinticBlend(const double ratio)
{
    const double t2 = ratio * ratio;
    const double t3 = t2 * ratio;
    return 10.0 * t3 - 15.0 * t3 * ratio +
           6.0 * t3 * t2;
}

void runSegment(
    const std::string &name,
    robot::ti5::HandController &controller,
    const robot::ti5::Hand::PositionValues &from,
    const robot::ti5::Hand::PositionValues &to,
    const robot::ti5::Hand::SpeedValues &speeds,
    const std::chrono::milliseconds duration)
{
    const auto cycles = std::max<std::size_t>(
        1,
        static_cast<std::size_t>(duration / kControlPeriod));
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
        controller.setTarget(interpolate(from, to, blend), speeds);
        controller.update();
        next += kControlPeriod;
        std::this_thread::sleep_until(next);
    }
}

void runOneHand(const std::string &name,
                const robot::ti5::HandSide side,
                const robot::ti5::hand::HandSideConfig &config,
                const std::string &interface_name,
                const std::chrono::milliseconds timeout,
                const Options &options)
{
    robot::ti5::Hand hand(side, config, interface_name, timeout);
    robot::ti5::HandController controller(hand);
    robot::ti5::Hand::SpeedValues speeds{};
    speeds.fill(options.speed_raw);

    try
    {
        controller.start(speeds);
        const auto start = controller.targetPositions();
        const auto target = makeTarget(start, options);
        runSegment(name + "移出", controller, start, target,
                   speeds, kMoveDuration);
        runSegment(name + "端点保持", controller, target, target,
                   speeds, kEndpointHoldDuration);
        if (!controller.targetReachedRaw(kReachedToleranceRaw))
        {
            throw std::runtime_error(
                name + "没有到达目标位置容差");
        }
        runSegment(name + "返回", controller, target, start,
                   speeds, kMoveDuration);
        runSegment(name + "返回点保持", controller, start, start,
                   speeds, kEndpointHoldDuration);
        if (!controller.targetReachedRaw(kReachedToleranceRaw))
        {
            throw std::runtime_error(
                name + "没有返回起始位置容差");
        }
        controller.pause();
        robot::common::logger()->info(
            "{}移动和返回通过；现已停止继续发送命令", name);
    }
    catch (...)
    {
        robot::common::logger()->warn(
            "{}异常后停止继续发送命令；协议没有已确认的停止命令，"
            "不能假定灵巧手已经释放", name);
        throw;
    }
}

void requireMotionConfirmation(const Options &options)
{
    std::cout
        << "\n即将测试" << selectionName(options.side) << "，";
    if (options.channel)
    {
        std::cout << "只移动原始通道 " << *options.channel;
    }
    else
    {
        std::cout << "六个原始通道一起移动";
    }
    std::cout
        << ' ' << options.delta_raw << " 个位置单位后返回，速度原始值 "
        << static_cast<unsigned int>(options.speed_raw) << "。\n"
        << "请确保手指周围没有夹持物和人体；返回后程序停止继续发送命令，"
           "但协议没有已确认的释放命令。\n"
        << "确认请输入 y 或 Y；其他输入取消：";
    std::string confirmation;
    std::getline(std::cin, confirmation);
    if (confirmation != "y" && confirmation != "Y")
    {
        throw std::runtime_error(
            "操作者取消；未发送灵巧手位置命令");
    }
}

} // namespace

int main(const int argc, char **argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try
    {
        const auto options = parseOptions(argc, argv);
        const auto can_config = robot::ti5::loadCanConfig(
            configPath("can.yaml"));
        auto hand_config = robot::ti5::hand::loadHandConfig(
            configPath("hands.yaml"));
        configureSelectedSides(hand_config, options);

        std::cout
            << "TI5 灵巧手独立实机检查计划："
            << selectionName(options.side);
        if (options.read_only)
        {
            std::cout << "，只读取状态。\n";
        }
        else
        {
            std::cout
                << "，位移原始值=" << options.delta_raw
                << "，速度原始值="
                << static_cast<unsigned int>(options.speed_raw)
                << "，通道="
                << (options.channel
                        ? std::to_string(*options.channel)
                        : std::string{"all"})
                << "。\n";
        }

        if (options.dry_run)
        {
            std::cout << "仅核对参数和配置完成；未打开 CAN。\n";
            return 0;
        }

        requireEnvironmentConfirmation();
        if (options.commission)
        {
            requireCommissionConfirmation();
        }
        if (::geteuid() == 0)
        {
            throw std::runtime_error(
                "请以普通用户运行，不要使用 sudo/root");
        }
        SingleProcessLock process_lock;
        requireExclusiveController();

        const auto mapping = discoverHandBuses(can_config, hand_config);
        if (mapping.left)
        {
            readOneHand(robot::ti5::HandSide::Left,
                        hand_config.left,
                        *mapping.left,
                        hand_config.discovery.response_timeout);
        }
        if (mapping.right)
        {
            readOneHand(robot::ti5::HandSide::Right,
                        hand_config.right,
                        *mapping.right,
                        hand_config.discovery.response_timeout);
        }

        if (options.read_only)
        {
            robot::common::logger()->info(
                "灵巧手接口发现和状态读取通过；未发送位置命令");
            return 0;
        }

        requireMotionConfirmation(options);
        if (mapping.left)
        {
            runOneHand("左手",
                       robot::ti5::HandSide::Left,
                       hand_config.left,
                       *mapping.left,
                       hand_config.discovery.response_timeout,
                       options);
        }
        if (mapping.right)
        {
            runOneHand("右手",
                       robot::ti5::HandSide::Right,
                       hand_config.right,
                       *mapping.right,
                       hand_config.discovery.response_timeout,
                       options);
        }

        robot::common::logger()->info(
            "TI5 灵巧手独立实机检查全部完成；未发送未经确认的停止命令");
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "TI5 灵巧手独立实机检查失败：{}", error.what());
        return 1;
    }
}
