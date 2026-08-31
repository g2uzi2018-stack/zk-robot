#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"
#include "ti5/arm/arm.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/config/config_loader.hpp"
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
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
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

constexpr double kDefaultDeltaRad = 0.01;
constexpr double kMinimumDeltaRad = 0.001;
constexpr double kMaximumDeltaRad = 0.02;
constexpr auto kMoveDuration = std::chrono::seconds{2};
constexpr auto kFailureHoldDuration = std::chrono::milliseconds{200};

std::atomic<bool> stop_requested{false};

void signalHandler(int)
{
    stop_requested.store(true);
}

struct Options
{
    robot::ti5::ArmSide side{robot::ti5::ArmSide::Left};
    std::string joint_name{"left_shoulder_pitch"};
    double delta_rad{kDefaultDeltaRad};
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
        << " --side left|right --joint 关节名称 "
           "[--delta-rad 数值] [--dry-run]\n\n"
        << "示例：\n"
        << "  " << program
        << " --side left --joint left_wrist_roll --delta-rad 0.01\n\n"
        << "工具会读取整臂、在当前位置建立保持、让所选关节平滑移动"
           "一个很小角度、回到原位，然后请求并确认 mode=0。\n"
        << "--dry-run 只核对参数和配置，不打开 CAN。\n";
}

Options parseOptions(const int argc, char **argv)
{
    Options options;
    bool side_set = false;
    bool joint_set = false;
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
        if (index + 1 >= argc)
        {
            throw std::invalid_argument(argument + " 缺少数值");
        }
        const std::string value{argv[++index]};
        if (argument == "--side")
        {
            if (value == "left")
            {
                options.side = robot::ti5::ArmSide::Left;
            }
            else if (value == "right")
            {
                options.side = robot::ti5::ArmSide::Right;
            }
            else
            {
                throw std::invalid_argument(
                    "--side 只能是 left 或 right");
            }
            side_set = true;
        }
        else if (argument == "--joint")
        {
            options.joint_name = value;
            joint_set = true;
        }
        else if (argument == "--delta-rad")
        {
            try
            {
                options.delta_rad = std::stod(value);
            }
            catch (const std::exception &)
            {
                throw std::invalid_argument(
                    "--delta-rad 不是有效数值");
            }
        }
        else
        {
            throw std::invalid_argument("未知参数：" + argument);
        }
    }
    if (!side_set || !joint_set)
    {
        throw std::invalid_argument(
            "实机检查必须显式提供 --side 和 --joint");
    }
    if (!std::isfinite(options.delta_rad) ||
        std::abs(options.delta_rad) < kMinimumDeltaRad ||
        std::abs(options.delta_rad) > kMaximumDeltaRad)
    {
        throw std::invalid_argument(
            "--delta-rad 的绝对值必须在 0.001～0.02 rad 之间");
    }
    const std::string prefix =
        options.side == robot::ti5::ArmSide::Left
            ? "left_"
            : "right_";
    if (options.joint_name.rfind(prefix, 0) != 0)
    {
        throw std::invalid_argument(
            "--joint 与 --side 指定的机械臂不一致");
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
    constexpr std::array<const char *, 4> names{
        "Ti5Control",
        "joint_manager",
        "ti5_follow_arm",
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
    const char *value = std::getenv("ZK_ROBOT_CONFIRM_ARM_CHECK");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "未设置 ZK_ROBOT_CONFIRM_ARM_CHECK=YES；未打开 CAN");
    }
}

std::vector<std::string> prepareBodyCan(
    const robot::ti5::CanConfig &can_config)
{
    robot::can::CanInterfaceManager manager;
    const auto body = manager.enumerate(
        can_config.socketcan.interface_regex);
    if (body.empty())
    {
        throw std::runtime_error(
            "没有找到符合 interface_regex 的 SocketCAN 接口");
    }

    const robot::can::CanInterfaceSettings settings{
        can_config.socketcan.bitrate,
        static_cast<std::uint32_t>(
            can_config.socketcan.restart_ms.count()),
        can_config.socketcan.reconfigure_wait,
        can_config.socketcan.startup_wait,
        can_config.socketcan.validate_bitrate};
    std::vector<std::string> result;
    for (const auto &interface : body)
    {
        const auto ready = manager.prepare(interface.name, settings);
        if (!ready.up)
        {
            throw std::runtime_error(
                interface.name + " 拉起失败");
        }
        result.push_back(ready.name);
    }
    return result;
}

std::pair<robot::ti5::LogicalCanBus, std::string> discoverArmBus(
    const robot::ti5::Ti5RobotConfig &robot_config,
    const robot::ti5::CanConfig &can_config,
    const robot::ti5::ArmSide side,
    const std::vector<std::string> &candidate_interfaces)
{
    const std::string name =
        side == robot::ti5::ArmSide::Left ? "left_arm" : "right_arm";
    const auto configured = std::find_if(
        robot_config.can_buses.begin(),
        robot_config.can_buses.end(),
        [&name](const auto &bus) { return bus.name == name; });
    if (configured == robot_config.can_buses.end())
    {
        throw std::runtime_error("robot.yaml 缺少逻辑总线：" + name);
    }

    robot::ti5::CanDiscovery discovery;
    const auto result = discovery.discover(
        {*configured},
        robot::ti5::makeDiscoveryOptions(can_config),
        candidate_interfaces);
    if (!result.success || result.logical_buses.size() != 1 ||
        !result.logical_buses.front().complete ||
        !result.logical_buses.front().interface_name)
    {
        throw std::runtime_error(name + " CAN 总线发现失败");
    }
    return {*configured, *result.logical_buses.front().interface_name};
}

double quinticBlend(const double ratio)
{
    const double t2 = ratio * ratio;
    const double t3 = t2 * ratio;
    return 10.0 * t3 - 15.0 * t3 * ratio +
           6.0 * t3 * t2;
}

void moveSmoothly(robot::ti5::Arm &arm,
                  const robot::ti5::Arm::JointValues &from,
                  const robot::ti5::Arm::JointValues &to,
                  robot::ti5::Arm::JointValues &last_target,
                  const std::chrono::milliseconds period)
{
    const auto cycles = static_cast<std::size_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(kMoveDuration)
            .count() /
        period.count());
    auto next = std::chrono::steady_clock::now();
    for (std::size_t cycle = 1; cycle <= cycles; ++cycle)
    {
        if (stop_requested.load())
        {
            throw std::runtime_error("操作者中止 Arm 实机检查");
        }
        const auto ratio = static_cast<double>(cycle) /
                           static_cast<double>(cycles);
        const auto blend = quinticBlend(ratio);
        for (std::size_t index = 0; index < arm.kJointCount; ++index)
        {
            last_target[index] =
                from[index] + (to[index] - from[index]) * blend;
        }
        arm.commandPositionsCsp(last_target);
        next += period;
        std::this_thread::sleep_until(next);
    }
}

void holdBestEffort(robot::ti5::Arm &arm,
                    const robot::ti5::Arm::JointValues &target,
                    const std::chrono::milliseconds period)
{
    const auto cycles = static_cast<std::size_t>(
        kFailureHoldDuration.count() / period.count());
    for (std::size_t cycle = 0; cycle < cycles; ++cycle)
    {
        for (std::size_t index = 0;
             index < robot::ti5::Arm::kJointCount;
             ++index)
        {
            try
            {
                arm.joint(index).commandPositionCsp(target[index]);
            }
            catch (const std::exception &)
            {
            }
        }
        std::this_thread::sleep_for(period);
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
        const auto robot_config = robot::ti5::loadRobotConfig(
            configPath("robot.yaml"));
        const auto can_config = robot::ti5::loadCanConfig(
            configPath("can.yaml"));
        const auto safety = robot::ti5::loadJointSafetyConfig(
            configPath("safety.yaml"));
        const auto kinematics = robot::ti5::loadKinematicsConfig(
            configPath("kinematics.yaml"));
        const std::string model_name =
            options.side == robot::ti5::ArmSide::Left
                ? "t7_t170_left_arm"
                : "t7_t170_right_arm";
        const auto model = kinematics.models.find(model_name);
        if (model == kinematics.models.end())
        {
            throw std::runtime_error(
                "kinematics.yaml 缺少模型：" + model_name);
        }
        const auto joint_configs = robot::ti5::makeJointConfigs(
            robot_config, safety, &model->second);
        const auto selected = std::find_if(
            joint_configs.begin(),
            joint_configs.end(),
            [&options](const auto &config)
            {
                return config.physical_joint.name == options.joint_name;
            });
        if (selected == joint_configs.end())
        {
            throw std::invalid_argument(
                "所选关节不属于指定机械臂：" + options.joint_name);
        }

        std::cout
            << "Arm 实机检查计划：" << model_name << "，关节 "
            << options.joint_name << "，模型坐标位移 "
            << options.delta_rad << " rad。\n";
        if (options.dry_run)
        {
            std::cout << "仅核对配置完成；未打开 CAN。\n";
            return 0;
        }

        requireExplicitConfirmation();
        SingleProcessLock process_lock;
        requireExclusiveController();
        const auto candidates = prepareBodyCan(can_config);
        const auto [logical_bus, interface_name] = discoverArmBus(
            robot_config, can_config, options.side, candidates);
        robot::common::logger()->info(
            "Arm 实机检查总线映射：{} -> {}",
            logical_bus.name,
            interface_name);

        auto bus = std::make_unique<robot::ti5::CanBus>(
            interface_name,
            robot::ti5::CanBusOptions{
                logical_bus.expected_node_ids,
                can_config.receive.use_can_filters,
                can_config.receive.receive_error_frames,
                can_config.control.send_failure_threshold});
        robot::ti5::ArmOptions arm_options;
        arm_options.control_period = std::chrono::milliseconds{
            static_cast<std::chrono::milliseconds::rep>(
                1000 / can_config.control.frequency_hz)};
        arm_options.inter_frame_gap =
            can_config.control.inter_frame_gap;
        arm_options.maximum_stale_cycles =
            can_config.watchdog.stale_feedback_cycles;
        robot::ti5::Arm arm(
            options.side,
            std::move(bus),
            joint_configs,
            arm_options);

        arm.prepare();
        const auto state = arm.readState();
        robot::ti5::Arm::JointValues start{};
        std::size_t selected_index = arm.kJointCount;
        std::cout << "\n整臂状态：\n";
        for (std::size_t index = 0; index < arm.kJointCount; ++index)
        {
            const auto &joint_state = state.joints[index];
            if (!joint_state.position_rad)
            {
                throw std::runtime_error(
                    arm.jointNames()[index] + " 缺少位置反馈");
            }
            start[index] = *joint_state.position_rad;
            if (arm.jointNames()[index] == options.joint_name)
            {
                selected_index = index;
            }
            std::cout
                << "  " << arm.jointNames()[index]
                << " position=" << start[index]
                << " rad mode="
                << (joint_state.run_mode
                        ? std::to_string(*joint_state.run_mode)
                        : "unknown")
                << " fault="
                << (joint_state.fault_bits
                        ? std::to_string(*joint_state.fault_bits)
                        : "unknown")
                << '\n';
        }
        if (selected_index >= arm.kJointCount)
        {
            throw std::logic_error("所选关节索引组装失败");
        }
        try
        {
            arm.validatePositions(start);
        }
        catch (const std::exception &error)
        {
            throw std::runtime_error(
                std::string{"当前位置不能建立 Arm 控制："} +
                error.what() +
                "；请先运行 ti5_zero_home 完成边界接管");
        }

        auto target = start;
        target[selected_index] += options.delta_rad;
        arm.validatePositions(target);

        std::cout
            << "\n接下来会在当前位置建立保持，将 "
            << options.joint_name << " 平滑移动 "
            << options.delta_rad
            << " rad，再回到原位。全过程请可靠托住机械臂；"
               "回原位后程序会请求并确认 mode=0。\n"
            << "确认请输入 y 或 Y；其他输入取消：";
        std::string confirmation;
        std::getline(std::cin, confirmation);
        if (confirmation != "y" && confirmation != "Y")
        {
            throw std::runtime_error("操作者取消；未发送 0x44 或 0x02");
        }

        bool stop_started = false;
        auto last_target = start;
        try
        {
            arm.startPositionControlAtCurrentPosition();
            moveSmoothly(
                arm,
                start,
                target,
                last_target,
                arm_options.control_period);
            moveSmoothly(
                arm,
                target,
                start,
                last_target,
                arm_options.control_period);

            stop_started = true;
            arm.requestStopModeAndConfirm();
            robot::common::logger()->info(
                "PASS：Arm 读取、当前位置保持、小幅运动、回原位和 STOP 确认全部完成");
        }
        catch (...)
        {
            if (arm.hasSentPositionCommand() && !stop_started)
            {
                robot::common::logger()->warn(
                    "Arm 检查异常；继续发送最后目标 0.20 秒，不自动发送 STOP");
                holdBestEffort(
                    arm, last_target, arm_options.control_period);
            }
            throw;
        }
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "TI5 Arm 实机检查失败：{}", error.what());
        return 1;
    }
}
