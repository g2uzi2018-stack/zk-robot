#include "input/exoskeleton/exoskeleton.hpp"
#include "tiago/arm/arm.hpp"
#include "tiago/base/base.hpp"
#include "tiago/base/base_config.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"
#include "tiago/executor/robot_control_executor.hpp"
#include "tiago/gripper/gripper.hpp"
#include "tiago/head/head.hpp"
#include "tiago/torso/torso.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace
{

using Exoskeleton = robot::input::exoskeleton::Exoskeleton;
using ExoskeletonState = robot::input::exoskeleton::ExoskeletonState;
using Executor = robot::tiago::RobotControlExecutor;
using Arm = robot::tiago::Arm;
using Gripper = robot::tiago::Gripper;

constexpr std::size_t kArmJointCount = Arm::kJointCount;

std::atomic<bool> stop_requested{false};

std::string modeHex(std::uint8_t value);

void signalHandler(int)
{
    stop_requested.store(true);
}

std::filesystem::path sourcePath(const char *relative_path)
{
#ifdef ZK_ROBOT_SOURCE_DIR
    return std::filesystem::path{ZK_ROBOT_SOURCE_DIR} / relative_path;
#else
    return std::filesystem::path{relative_path};
#endif
}

struct Options
{
    std::filesystem::path exoskeleton_config{
        sourcePath("config/exoskeleton.yaml")};
    std::filesystem::path tiago_root{sourcePath("config/tiago")};
    std::filesystem::path teleop_config{
        sourcePath("config/exoskeleton_tiago_teleop.yaml")};
    bool confirm{false};
    bool dry_run{false};
    bool clear_fault{false};
};

void printUsage(const char *program)
{
    std::cout
        << "用法：\n"
        << "  " << program
        << " [--exoskeleton-config PATH] [--tiago-root PATH]\n"
           "      [--teleop-config PATH] [--confirm] [--clear-fault]\n"
           "      [--dry-run]\n\n"
        << "默认只做配置检查，不打开 TIAGo CAN。\n"
        << "真实遥操作必须显式传 --confirm；它仍会等待外骨骼模式、\n"
           "机器人反馈、初始姿态接管和左轮盘回中全部满足后才使能电机。\n\n"
        << "仿真示例：\n"
        << "  " << program << " --confirm\n\n"
        << "真机示例：\n"
        << "  sudo " << program
        << " --tiago-root /path/to/real-tiago-config --confirm\n\n"
        << "运行中按 Ctrl-C 会先发底盘零速，再停止并禁用已使能的电机。\n";
}

Options parseOptions(int argc, char **argv)
{
    Options options;

    for (int index = 1; index < argc; ++index)
    {
        const std::string argument{argv[index]};
        if (argument == "--help" || argument == "-h")
        {
            printUsage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (argument == "--confirm")
        {
            options.confirm = true;
            continue;
        }
        if (argument == "--dry-run")
        {
            options.dry_run = true;
            continue;
        }
        if (argument == "--clear-fault")
        {
            options.clear_fault = true;
            continue;
        }

        if (index + 1 >= argc)
        {
            throw std::invalid_argument(argument + " 缺少参数");
        }
        const std::filesystem::path value{argv[++index]};
        if (argument == "--exoskeleton-config")
        {
            options.exoskeleton_config = value;
        }
        else if (argument == "--tiago-root")
        {
            options.tiago_root = value;
        }
        else if (argument == "--teleop-config")
        {
            options.teleop_config = value;
        }
        else
        {
            throw std::invalid_argument("未知参数: " + argument);
        }
    }

    if (options.dry_run)
    {
        options.confirm = false;
    }
    return options;
}

[[noreturn]] void throwConfigError(
    const std::string &context,
    const std::string &message)
{
    throw std::invalid_argument(context + ": " + message);
}

YAML::Node requireMap(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsMap())
    {
        throwConfigError(context + "." + key, "必须是 YAML mapping");
    }
    return value;
}

YAML::Node requireValue(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsScalar())
    {
        throwConfigError(context + "." + key, "必须是 YAML scalar");
    }
    return value;
}

template <typename T>
T requireScalar(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    try
    {
        return requireValue(parent, key, context).as<T>();
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            context + "." + key,
            "类型错误: " + std::string(error.what()));
    }
}

double requireFiniteDouble(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const double value = requireScalar<double>(parent, key, context);
    if (!std::isfinite(value))
    {
        throwConfigError(context + "." + key, "必须是有限数值");
    }
    return value;
}

std::chrono::milliseconds requireMilliseconds(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = requireScalar<std::int64_t>(parent, key, context);
    if (value <= 0)
    {
        throwConfigError(context + "." + key, "必须是正整数毫秒");
    }
    return std::chrono::milliseconds{value};
}

std::uint8_t requireByte(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto text = requireScalar<std::string>(parent, key, context);
    try
    {
        std::size_t consumed = 0;
        const auto value = std::stoul(text, &consumed, 0);
        if (consumed != text.size() || value > 0xFFU)
        {
            throwConfigError(
                context + "." + key,
                "必须是 0..255 的十进制或 0x 前缀数值");
        }
        return static_cast<std::uint8_t>(value);
    }
    catch (const std::invalid_argument &)
    {
        throwConfigError(
            context + "." + key,
            "必须是 0..255 的十进制或 0x 前缀数值");
    }
    catch (const std::out_of_range &)
    {
        throwConfigError(
            context + "." + key,
            "数值超出范围");
    }
}

int requireSign(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const int value = requireScalar<int>(parent, key, context);
    if (value != 1 && value != -1)
    {
        throwConfigError(context + "." + key, "只能是 1 或 -1");
    }
    return value;
}

double requireFraction(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const double value = requireFiniteDouble(parent, key, context);
    if (value <= 0.0 || value > 1.0)
    {
        throwConfigError(context + "." + key, "必须满足 0 < value <= 1");
    }
    return value;
}

template <std::size_t N>
std::array<double, N> requireDoubleArray(
    const YAML::Node &parent,
    const char *key,
    const std::string &context)
{
    const auto value = parent[key];
    if (!value || !value.IsSequence() || value.size() != N)
    {
        throwConfigError(
            context + "." + key,
            "必须是长度为 " + std::to_string(N) + " 的数组");
    }

    std::array<double, N> result{};
    for (std::size_t index = 0; index < N; ++index)
    {
        try
        {
            result[index] = value[index].as<double>();
        }
        catch (const YAML::Exception &error)
        {
            throwConfigError(
                context + "." + key + "[" + std::to_string(index) + "]",
                "类型错误: " + std::string(error.what()));
        }
        if (!std::isfinite(result[index]))
        {
            throwConfigError(
                context + "." + key + "[" + std::to_string(index) + "]",
                "必须是有限数值");
        }
    }
    return result;
}

struct TeleopConfig
{
    std::uint8_t run_mode_raw{0};
    std::uint8_t exit_mode_raw{0};

    std::chrono::milliseconds executor_control_period{};
    std::chrono::milliseconds loop_period{};
    std::chrono::milliseconds command_timeout{};
    std::chrono::milliseconds robot_feedback_timeout{};

    std::array<double, kArmJointCount> left_scale{};
    std::array<double, kArmJointCount> left_offset_rad{};
    std::array<double, kArmJointCount> right_scale{};
    std::array<double, kArmJointCount> right_offset_rad{};

    double arm_velocity_fraction{0.0};
    double initial_sync_tolerance_rad{0.0};

    double open_finger_position_m{0.0};
    double closed_finger_position_m{0.0};
    double gripper_velocity_fraction{0.0};
    double initial_gripper_tolerance_m{0.0};

    double base_linear_velocity_fraction{0.0};
    double base_angular_velocity_fraction{0.0};
    int linear_axis_sign{1};
    int angular_axis_sign{1};
    double rearm_center_threshold{0.0};

    std::string lock_file;
};

TeleopConfig loadTeleopConfig(const std::filesystem::path &path)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path.string());
    }
    catch (const YAML::Exception &error)
    {
        throwConfigError(
            path.string(),
            "无法读取或解析 YAML: " + std::string(error.what()));
    }
    if (!root || !root.IsMap())
    {
        throwConfigError(path.string(), "根节点必须是 YAML mapping");
    }

    const auto teleop = requireMap(root, "teleop", path.string());
    const auto context = path.string() + ".teleop";
    const auto mode = requireMap(teleop, "mode", context);
    const auto mode_context = context + ".mode";
    const auto timing = requireMap(teleop, "timing", context);
    const auto timing_context = context + ".timing";
    const auto retargeting = requireMap(teleop, "retargeting", context);
    const auto retargeting_context = context + ".retargeting";
    const auto arm = requireMap(teleop, "arm", context);
    const auto arm_context = context + ".arm";
    const auto gripper = requireMap(teleop, "gripper", context);
    const auto gripper_context = context + ".gripper";
    const auto base = requireMap(teleop, "base", context);
    const auto base_context = context + ".base";
    const auto safety = requireMap(teleop, "safety", context);
    const auto safety_context = context + ".safety";

    TeleopConfig result;
    result.run_mode_raw = requireByte(mode, "run_raw", mode_context);
    result.exit_mode_raw = requireByte(mode, "exit_raw", mode_context);
    if (result.run_mode_raw == result.exit_mode_raw)
    {
        throwConfigError(mode_context, "run_raw 和 exit_raw 不能相同");
    }

    result.executor_control_period = requireMilliseconds(
        timing, "executor_control_period_ms", timing_context);
    result.loop_period = requireMilliseconds(
        timing, "loop_period_ms", timing_context);
    result.command_timeout = requireMilliseconds(
        timing, "command_timeout_ms", timing_context);
    result.robot_feedback_timeout = requireMilliseconds(
        timing, "robot_feedback_timeout_ms", timing_context);
    if (result.command_timeout <= result.loop_period)
    {
        throwConfigError(
            timing_context,
            "command_timeout_ms 必须大于 loop_period_ms");
    }

    result.left_scale = requireDoubleArray<kArmJointCount>(
        retargeting, "left_scale", retargeting_context);
    result.left_offset_rad = requireDoubleArray<kArmJointCount>(
        retargeting, "left_offset_rad", retargeting_context);
    result.right_scale = requireDoubleArray<kArmJointCount>(
        retargeting, "right_scale", retargeting_context);
    result.right_offset_rad = requireDoubleArray<kArmJointCount>(
        retargeting, "right_offset_rad", retargeting_context);

    result.arm_velocity_fraction = requireFraction(
        arm, "velocity_fraction", arm_context);
    result.initial_sync_tolerance_rad = requireFiniteDouble(
        arm, "initial_sync_tolerance_rad", arm_context);
    if (result.initial_sync_tolerance_rad <= 0.0)
    {
        throwConfigError(
            arm_context + ".initial_sync_tolerance_rad",
            "必须是正数");
    }

    result.open_finger_position_m = requireFiniteDouble(
        gripper, "open_position_m", gripper_context);
    result.closed_finger_position_m = requireFiniteDouble(
        gripper, "closed_position_m", gripper_context);
    if (result.closed_finger_position_m >= result.open_finger_position_m)
    {
        throwConfigError(
            gripper_context,
            "closed_position_m 必须小于 open_position_m");
    }
    result.gripper_velocity_fraction = requireFraction(
        gripper, "velocity_fraction", gripper_context);
    result.initial_gripper_tolerance_m = requireFiniteDouble(
        gripper, "initial_sync_tolerance_m", gripper_context);
    if (result.initial_gripper_tolerance_m <= 0.0)
    {
        throwConfigError(
            gripper_context + ".initial_sync_tolerance_m",
            "必须是正数");
    }

    result.base_linear_velocity_fraction = requireFraction(
        base, "linear_velocity_fraction", base_context);
    result.base_angular_velocity_fraction = requireFraction(
        base, "angular_velocity_fraction", base_context);
    result.linear_axis_sign = requireSign(
        base, "linear_axis_sign", base_context);
    result.angular_axis_sign = requireSign(
        base, "angular_axis_sign", base_context);
    result.rearm_center_threshold = requireFiniteDouble(
        base, "rearm_center_threshold", base_context);
    if (result.rearm_center_threshold <= 0.0 ||
        result.rearm_center_threshold > 1.0)
    {
        throwConfigError(
            base_context + ".rearm_center_threshold",
            "必须满足 0 < value <= 1");
    }

    result.lock_file = requireScalar<std::string>(
        safety, "lock_file", safety_context);
    if (result.lock_file.empty())
    {
        throwConfigError(safety_context + ".lock_file", "不能为空");
    }

    return result;
}

struct TiagoConfigs
{
    robot::tiago::CanBusConfig left_shoulder;
    robot::tiago::CanBusConfig left_elbow;
    robot::tiago::CanBusConfig left_wrist;
    robot::tiago::CanBusConfig right_shoulder;
    robot::tiago::CanBusConfig right_elbow;
    robot::tiago::CanBusConfig right_wrist;
    robot::tiago::CanBusConfig left_gripper;
    robot::tiago::CanBusConfig right_gripper;
    robot::tiago::CanBusConfig head;
    robot::tiago::CanBusConfig torso;
    robot::tiago::BaseConfig base;
};

TiagoConfigs loadTiagoConfigs(const std::filesystem::path &root)
{
    const auto can = root / "can";
    TiagoConfigs result{
        robot::tiago::loadCanBusConfig(can / "left_shoulder.yaml"),
        robot::tiago::loadCanBusConfig(can / "left_elbow.yaml"),
        robot::tiago::loadCanBusConfig(can / "left_wrist.yaml"),
        robot::tiago::loadCanBusConfig(can / "right_shoulder.yaml"),
        robot::tiago::loadCanBusConfig(can / "right_elbow.yaml"),
        robot::tiago::loadCanBusConfig(can / "right_wrist.yaml"),
        robot::tiago::loadCanBusConfig(can / "left_gripper.yaml"),
        robot::tiago::loadCanBusConfig(can / "right_gripper.yaml"),
        robot::tiago::loadCanBusConfig(can / "head.yaml"),
        robot::tiago::loadCanBusConfig(can / "torso.yaml"),
        robot::tiago::loadBaseConfig(root / "base" / "base.yaml")};
    return result;
}

class SingleProcessLock final
{
public:
    explicit SingleProcessLock(const std::string &path)
    {
        fd_ = ::open(
            path.c_str(),
            O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW,
            0666);
        if (fd_ < 0)
        {
            throw std::runtime_error(
                "无法打开 TIAGo 遥操作互斥锁 " + path);
        }
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
        {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "另一个 TIAGo 运动程序正在运行，拒绝启动遥操作");
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

template <typename Function>
void bestEffort(const char *operation, Function &&function) noexcept
{
    try
    {
        function();
    }
    catch (const std::exception &error)
    {
        std::cerr << "清理动作失败 (" << operation << "): "
                  << error.what() << '\n';
    }
    catch (...)
    {
        std::cerr << "清理动作失败 (" << operation << "): unknown error\n";
    }
}

class HardwareGuard final
{
public:
    HardwareGuard(
        Executor &executor,
        Arm &left_arm,
        Arm &right_arm,
        Gripper &left_gripper,
        Gripper &right_gripper,
        robot::tiago::Base &base)
        : executor_(executor),
          left_arm_(left_arm),
          right_arm_(right_arm),
          left_gripper_(left_gripper),
          right_gripper_(right_gripper),
          base_(base)
    {
    }

    ~HardwareGuard()
    {
        stopAndDisable();
    }

    void markExecutorRunning() noexcept
    {
        executor_running_ = true;
    }

    void markHardwareEnableStarted() noexcept
    {
        // 只要进入 enable 流程，异常清理就尝试对所有相关节点执行
        // stop + disable；对尚未 enable 的节点发送 stop/disable 也是安全的。
        hardware_enable_started_ = true;
    }

    void shutdownExecutor() noexcept
    {
        if (!executor_running_)
        {
            return;
        }

        bestEffort("发送底盘零速", [&] {
            executor_.setBaseVelocity(0.0, 0.0);
        });
        bestEffort("停止 Executor", [&] {
            executor_.shutdown();
        });
        executor_running_ = false;
    }

    void stopAndDisable() noexcept
    {
        shutdownExecutor();
        if (!hardware_enable_started_)
        {
            return;
        }

        // 先停底盘，减少实体机器人仍在移动的时间窗口。
        bestEffort("停止底盘", [&] { base_.stop(); });
        bestEffort("停止左臂", [&] { left_arm_.stop(); });
        bestEffort("停止右臂", [&] { right_arm_.stop(); });
        bestEffort("停止左夹爪", [&] { left_gripper_.stop(); });
        bestEffort("停止右夹爪", [&] { right_gripper_.stop(); });

        bestEffort("禁用底盘", [&] { base_.disable(); });
        bestEffort("禁用左臂", [&] { left_arm_.disable(); });
        bestEffort("禁用右臂", [&] { right_arm_.disable(); });
        bestEffort("禁用左夹爪", [&] { left_gripper_.disable(); });
        bestEffort("禁用右夹爪", [&] { right_gripper_.disable(); });

        hardware_enable_started_ = false;
    }

private:
    Executor &executor_;
    Arm &left_arm_;
    Arm &right_arm_;
    Gripper &left_gripper_;
    Gripper &right_gripper_;
    robot::tiago::Base &base_;
    bool executor_running_{false};
    bool hardware_enable_started_{false};
};

template <std::size_t N>
bool copyCompletePositions(
    const std::array<std::optional<double>, N> &source,
    std::array<double, N> &destination)
{
    for (std::size_t index = 0; index < N; ++index)
    {
        if (!source[index] || !std::isfinite(*source[index]))
        {
            return false;
        }
        destination[index] = *source[index];
    }
    return true;
}

struct InitialFeedback
{
    Arm::JointValues left_arm{};
    Arm::JointValues right_arm{};
    Gripper::FingerValues left_gripper{};
    Gripper::FingerValues right_gripper{};
};

bool extractInitialFeedback(
    const Executor::RobotState &state,
    InitialFeedback &feedback)
{
    return copyCompletePositions(state.left_arm_positions, feedback.left_arm) &&
           copyCompletePositions(state.right_arm_positions, feedback.right_arm) &&
           copyCompletePositions(
               state.left_gripper_positions,
               feedback.left_gripper) &&
           copyCompletePositions(
               state.right_gripper_positions,
               feedback.right_gripper);
}

Executor::RobotState makeRobotState(const InitialFeedback &feedback)
{
    Executor::RobotState state;
    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        state.left_arm_positions[index] = feedback.left_arm[index];
        state.right_arm_positions[index] = feedback.right_arm[index];
    }
    for (std::size_t index = 0; index < Gripper::kFingerCount; ++index)
    {
        state.left_gripper_positions[index] = feedback.left_gripper[index];
        state.right_gripper_positions[index] = feedback.right_gripper[index];
    }
    return state;
}

void queryArmFeedback(Arm &arm)
{
    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        static_cast<void>(arm.joint(index).queryStatus());
    }
}

void queryGripperFeedback(Gripper &gripper)
{
    for (std::size_t index = 0; index < Gripper::kFingerCount; ++index)
    {
        static_cast<void>(gripper.finger(index).queryStatus());
    }
}

void waitForRobotFeedback(
    Arm &left_arm,
    Arm &right_arm,
    Gripper &left_gripper,
    Gripper &right_gripper,
    std::chrono::milliseconds timeout,
    InitialFeedback &feedback)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!stop_requested.load() &&
           std::chrono::steady_clock::now() < deadline)
    {
        queryArmFeedback(left_arm);
        queryArmFeedback(right_arm);
        queryGripperFeedback(left_gripper);
        queryGripperFeedback(right_gripper);

        InitialFeedback candidate;
        if (copyCompletePositions(left_arm.readPositions(), candidate.left_arm) &&
            copyCompletePositions(right_arm.readPositions(), candidate.right_arm) &&
            copyCompletePositions(
                left_gripper.readPositions(),
                candidate.left_gripper) &&
            copyCompletePositions(
                right_gripper.readPositions(),
                candidate.right_gripper))
        {
            feedback = candidate;
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }

    if (stop_requested.load())
    {
        throw std::runtime_error("收到停止请求，未启动 TIAGo 遥操作");
    }
    throw std::runtime_error(
        "等待 TIAGo 双臂和双夹爪反馈超时；未使能任何运动电机");
}

Arm::JointValues mapArm(
    const std::array<double, kArmJointCount> &source,
    const std::array<double, kArmJointCount> &scale,
    const std::array<double, kArmJointCount> &offset)
{
    Arm::JointValues result{};
    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        result[index] = source[index] * scale[index] + offset[index];
        if (!std::isfinite(result[index]))
        {
            throw std::runtime_error("外骨骼重定向产生非有限关节目标");
        }
    }
    return result;
}

double mapTrigger(double trigger, const TeleopConfig &config)
{
    const double normalized = std::clamp(trigger, 0.0, 1.0);
    return config.closed_finger_position_m +
           (1.0 - normalized) *
               (config.open_finger_position_m -
                config.closed_finger_position_m);
}

Gripper::FingerValues mapGripper(
    double trigger,
    const TeleopConfig &config)
{
    const double finger_position = mapTrigger(trigger, config);
    return Gripper::FingerValues{finger_position, finger_position};
}

struct MappedTargets
{
    Arm::JointValues left_arm{};
    Arm::JointValues right_arm{};
    Gripper::FingerValues left_gripper{};
    Gripper::FingerValues right_gripper{};
    double linear_velocity{0.0};
    double angular_velocity{0.0};
};

MappedTargets mapState(
    const ExoskeletonState &state,
    const TeleopConfig &config,
    const robot::tiago::BaseConfig &base_config)
{
    MappedTargets result;
    result.left_arm = mapArm(
        state.left_joint_rad,
        config.left_scale,
        config.left_offset_rad);
    result.right_arm = mapArm(
        state.right_joint_rad,
        config.right_scale,
        config.right_offset_rad);
    result.left_gripper = mapGripper(state.left.trigger, config);
    result.right_gripper = mapGripper(state.right.trigger, config);

    const double joystick_x = std::clamp(state.left.x, -1.0, 1.0);
    const double joystick_y = std::clamp(state.left.y, -1.0, 1.0);
    result.linear_velocity =
        static_cast<double>(config.linear_axis_sign) * joystick_y *
        config.base_linear_velocity_fraction * base_config.max_linear_velocity;
    result.angular_velocity =
        static_cast<double>(config.angular_axis_sign) * joystick_x *
        config.base_angular_velocity_fraction * base_config.max_angular_velocity;
    return result;
}

Arm::JointValues makeArmVelocityLimits(
    const robot::tiago::CanBusConfig &shoulder,
    const robot::tiago::CanBusConfig &elbow,
    const robot::tiago::CanBusConfig &wrist,
    double fraction)
{
    const std::array<double, kArmJointCount> maximums{
        shoulder.joints[0].limits.max_velocity,
        shoulder.joints[1].limits.max_velocity,
        elbow.joints[0].limits.max_velocity,
        elbow.joints[1].limits.max_velocity,
        wrist.joints[0].limits.max_velocity,
        wrist.joints[1].limits.max_velocity,
        wrist.joints[2].limits.max_velocity};

    Arm::JointValues result{};
    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        result[index] = maximums[index] * fraction;
    }
    return result;
}

Gripper::FingerValues makeGripperVelocityLimits(
    const robot::tiago::CanBusConfig &config,
    double fraction)
{
    return Gripper::FingerValues{
        config.joints[0].limits.max_velocity * fraction,
        config.joints[1].limits.max_velocity * fraction};
}

void validateMappedTargets(
    const MappedTargets &targets,
    Arm &left_arm,
    Arm &right_arm,
    Gripper &left_gripper,
    Gripper &right_gripper,
    robot::tiago::Base &base,
    const Arm::JointValues &left_arm_velocity_limits,
    const Arm::JointValues &right_arm_velocity_limits,
    const Gripper::FingerValues &left_gripper_velocity_limits,
    const Gripper::FingerValues &right_gripper_velocity_limits)
{
    std::vector<std::string> violations;
    const auto validate_joint = [&violations](
                                    const char *side,
                                    robot::tiago::Joint &joint,
                                    const double target,
                                    const double velocity_limit)
    {
        try
        {
            joint.validateCommand(target, velocity_limit);
        }
        catch (const std::exception &error)
        {
            std::ostringstream message;
            message << side << ' ' << joint.name() << ": " << error.what();
            violations.push_back(message.str());
        }
    };

    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        validate_joint(
            "左臂",
            left_arm.joint(index),
            targets.left_arm[index],
            left_arm_velocity_limits[index]);
        validate_joint(
            "右臂",
            right_arm.joint(index),
            targets.right_arm[index],
            right_arm_velocity_limits[index]);
    }
    for (std::size_t index = 0; index < Gripper::kFingerCount; ++index)
    {
        validate_joint(
            "左夹爪",
            left_gripper.finger(index),
            targets.left_gripper[index],
            left_gripper_velocity_limits[index]);
        validate_joint(
            "右夹爪",
            right_gripper.finger(index),
            targets.right_gripper[index],
            right_gripper_velocity_limits[index]);
    }

    try
    {
        base.validateVelocityCommand(
            targets.linear_velocity,
            targets.angular_velocity);
    }
    catch (const std::exception &error)
    {
        violations.push_back(std::string{"底盘: "} + error.what());
    }

    if (!violations.empty())
    {
        std::ostringstream message;
        message << "发现 " << violations.size() << " 项目标校验错误:";
        for (const auto &violation : violations)
        {
            message << "\n  - " << violation;
        }
        throw std::runtime_error(message.str());
    }
}

bool joystickCentered(
    const robot::input::exoskeleton::JoystickState &joystick,
    double threshold)
{
    return std::abs(joystick.x) <= threshold &&
           std::abs(joystick.y) <= threshold;
}

std::string readinessReason(
    const bool state_fresh,
    const ExoskeletonState &state,
    const Executor::RobotState &robot_state,
    const TeleopConfig &config,
    const robot::tiago::BaseConfig &base_config,
    Arm &left_arm,
    Arm &right_arm,
    Gripper &left_gripper,
    Gripper &right_gripper,
    robot::tiago::Base &base,
    const Arm::JointValues &left_arm_velocity_limits,
    const Arm::JointValues &right_arm_velocity_limits,
    const Gripper::FingerValues &left_gripper_velocity_limits,
    const Gripper::FingerValues &right_gripper_velocity_limits)
{
    if (!state_fresh)
    {
        return "外骨骼数据未连接或已过期";
    }
    if (state.left_mode_raw != config.run_mode_raw ||
        state.right_mode_raw != config.run_mode_raw)
    {
        return "等待左右外骨骼模式均为遥操作档位";
    }
    if (!joystickCentered(state.left, config.rearm_center_threshold))
    {
        return "等待左轮盘回中";
    }

    InitialFeedback current;
    if (!extractInitialFeedback(robot_state, current))
    {
        return "等待 TIAGo 双臂和双夹爪完整反馈";
    }

    MappedTargets targets;
    try
    {
        targets = mapState(state, config, base_config);
        validateMappedTargets(
            targets,
            left_arm,
            right_arm,
            left_gripper,
            right_gripper,
            base,
            left_arm_velocity_limits,
            right_arm_velocity_limits,
            left_gripper_velocity_limits,
            right_gripper_velocity_limits);
    }
    catch (const std::exception &error)
    {
        return "重定向目标未通过 TIAGo 限位检查: " +
               std::string(error.what());
    }

    for (std::size_t index = 0; index < kArmJointCount; ++index)
    {
        if (std::abs(targets.left_arm[index] - current.left_arm[index]) >
            config.initial_sync_tolerance_rad)
        {
            return "左臂初始接管未对齐，joint[" + std::to_string(index) +
                   "] 误差=" +
                   std::to_string(
                       std::abs(targets.left_arm[index] - current.left_arm[index])) +
                   " rad";
        }
        if (std::abs(targets.right_arm[index] - current.right_arm[index]) >
            config.initial_sync_tolerance_rad)
        {
            return "右臂初始接管未对齐，joint[" + std::to_string(index) +
                   "] 误差=" +
                   std::to_string(
                       std::abs(targets.right_arm[index] - current.right_arm[index])) +
                   " rad";
        }
    }
    for (std::size_t index = 0; index < Gripper::kFingerCount; ++index)
    {
        if (std::abs(targets.left_gripper[index] - current.left_gripper[index]) >
            config.initial_gripper_tolerance_m)
        {
            return "左夹爪初始接管未对齐，请调整左扳机";
        }
        if (std::abs(targets.right_gripper[index] - current.right_gripper[index]) >
            config.initial_gripper_tolerance_m)
        {
            return "右夹爪初始接管未对齐，请调整右扳机";
        }
    }

    return {};
}

std::string singleLine(std::string value)
{
    std::string result;
    result.reserve(value.size());
    bool previous_was_separator = false;
    for (const char character : value)
    {
        if (character == '\r' || character == '\n')
        {
            if (!previous_was_separator)
            {
                result += " | ";
                previous_was_separator = true;
            }
            continue;
        }
        result.push_back(character);
        previous_was_separator = false;
    }
    return result;
}

std::string panelLine(
    const char *label,
    const char *status,
    const std::string &detail)
{
    std::ostringstream output;
    output << label << " [" << status << "] " << singleLine(detail);
    return output.str();
}

class StatusPanel final
{
public:
    StatusPanel() : interactive_(::isatty(STDOUT_FILENO) != 0)
    {
    }

    ~StatusPanel()
    {
        finish();
    }

    void update(const std::vector<std::string> &lines)
    {
        if (lines.empty() || lines == lines_)
        {
            return;
        }

        if (interactive_)
        {
            if (lines_.empty())
            {
                // 禁止自动换行，保证每个状态只占终端的一行。
                output_ << "\033[?7l";
            }
            else
            {
                output_ << "\033[" << lines_.size() << 'A';
            }

            for (std::size_t index = 0; index < lines.size(); ++index)
            {
                output_ << '\r' << "\033[2K" << lines[index];
                if (index + 1 < lines.size())
                {
                    output_ << '\n';
                }
            }
            output_ << '\n' << std::flush;
        }
        else
        {
            // 重定向到文件或管道时没有光标可移动，只保留首个状态快照；
            // 运行时异常仍由外层错误日志输出。
            if (!lines_.empty())
            {
                return;
            }
            for (const auto &line : lines)
            {
                output_ << line << '\n';
            }
            output_ << std::flush;
        }

        lines_ = lines;
    }

    void finish() noexcept
    {
        if (finished_)
        {
            return;
        }
        finished_ = true;
        if (interactive_ && !lines_.empty())
        {
            // 恢复终端自动换行，并把后续错误/退出信息放到面板下方。
            output_ << "\033[?7h\n" << std::flush;
        }
    }

private:
    bool interactive_{false};
    bool finished_{false};
    std::vector<std::string> lines_;
    std::ostream &output_{std::cout};
};

std::string componentDetail(
    const std::string &reason,
    const std::string &component)
{
    const std::string bullet = "- " + component;
    const auto bullet_position = reason.find(bullet);
    if (bullet_position != std::string::npos)
    {
        const auto line_end = reason.find('\n', bullet_position);
        const auto detail_start = bullet_position + 2;
        return reason.substr(
            detail_start,
            line_end == std::string::npos
                ? std::string::npos
                : line_end - detail_start);
    }

    const auto component_position = reason.find(component);
    if (component_position != std::string::npos)
    {
        return reason.substr(component_position);
    }
    return reason;
}

std::string componentStatusLine(
    const char *label,
    const std::string &reason,
    const std::string &component)
{
    if (reason.empty())
    {
        return panelLine(label, "OK", "ready");
    }
    if (reason.find(component) != std::string::npos)
    {
        return panelLine(
            label,
            "ERROR",
            componentDetail(reason, component));
    }
    return panelLine(label, "WAIT", "等待其它就绪条件");
}

std::vector<std::string> readinessPanelLines(
    Exoskeleton &exoskeleton,
    const std::optional<ExoskeletonState> &state,
    const Executor::RobotState &robot_state,
    const TeleopConfig &config,
    const std::string &reason)
{
    std::vector<std::string> lines;
    lines.emplace_back("========== 外骨骼 -> TIAGo 遥操作状态 ==========");

    if (!state)
    {
        lines.push_back(panelLine(
            "外骨骼",
            "WAIT",
            "等待首个完整合法帧"));
        lines.push_back(panelLine(
            "模式",
            "WAIT",
            "等待外骨骼数据"));
        lines.push_back(panelLine(
            "左轮盘",
            "WAIT",
            "等待外骨骼数据"));
    }
    else
    {
        const bool fresh = exoskeleton.stateFresh();
        lines.push_back(panelLine(
            "外骨骼",
            fresh ? "OK" : "ERROR",
            std::string{fresh ? "connected | " : "stale | "} +
                "device=" + exoskeleton.device()));

        const bool mode_ready =
            fresh && state->left_mode_raw == config.run_mode_raw &&
            state->right_mode_raw == config.run_mode_raw;
        std::ostringstream mode_detail;
        mode_detail << "left=" << modeHex(state->left_mode_raw)
                    << ", right=" << modeHex(state->right_mode_raw);
        if (!mode_ready)
        {
            mode_detail << ", need=" << modeHex(config.run_mode_raw);
        }
        lines.push_back(panelLine(
            "模式",
            mode_ready ? "OK" : "WAIT",
            mode_detail.str()));

        const bool centered = joystickCentered(
            state->left,
            config.rearm_center_threshold);
        std::ostringstream joystick_detail;
        joystick_detail << std::fixed << std::setprecision(3)
                        << "x=" << state->left.x
                        << ", y=" << state->left.y;
        if (!centered)
        {
            joystick_detail << ", 请回中";
        }
        lines.push_back(panelLine(
            "左轮盘",
            centered ? "OK" : "WAIT",
            joystick_detail.str()));
    }

    InitialFeedback feedback;
    const bool feedback_ready = extractInitialFeedback(robot_state, feedback);
    lines.push_back(panelLine(
        "TIAGo反馈",
        feedback_ready ? "OK" : "WAIT",
        feedback_ready ? "双臂/双夹爪反馈完整" : "等待双臂和双夹爪反馈"));

    lines.push_back(componentStatusLine("左臂", reason, "左臂"));
    lines.push_back(componentStatusLine("右臂", reason, "右臂"));
    lines.push_back(componentStatusLine("左夹爪", reason, "左夹爪"));
    lines.push_back(componentStatusLine("右夹爪", reason, "右夹爪"));
    lines.push_back(componentStatusLine("底盘", reason, "底盘"));

    const bool error = reason.find("未通过") != std::string::npos ||
                       reason.find("未对齐") != std::string::npos ||
                       reason.find("故障") != std::string::npos;
    lines.push_back(panelLine(
        "总体",
        reason.empty() ? "OK" : (error ? "ERROR" : "WAIT"),
        reason.empty() ? "READY | 等待使能" : reason));
    return lines;
}

std::vector<std::string> runningPanelLines(
    Exoskeleton &exoskeleton,
    const ExoskeletonState &state,
    const MappedTargets &targets)
{
    std::ostringstream left_arm_detail;
    left_arm_detail << std::fixed << std::setprecision(3)
                    << "SERVO active | q0=" << targets.left_arm[0]
                    << ", q6=" << targets.left_arm[6];
    std::ostringstream right_arm_detail;
    right_arm_detail << std::fixed << std::setprecision(3)
                     << "SERVO active | q0=" << targets.right_arm[0]
                     << ", q6=" << targets.right_arm[6];
    std::ostringstream base_detail;
    base_detail << std::fixed << std::setprecision(3)
                << "v=" << targets.linear_velocity
                << " m/s, w=" << targets.angular_velocity << " rad/s";
    std::ostringstream mode_detail;
    mode_detail << "left=" << modeHex(state.left_mode_raw)
                << ", right=" << modeHex(state.right_mode_raw);
    std::ostringstream gripper_detail;
    gripper_detail << std::fixed << std::setprecision(3)
                   << "trigger=" << state.left.trigger;
    const std::string left_gripper_detail = gripper_detail.str();
    gripper_detail.str(std::string{});
    gripper_detail.clear();
    gripper_detail << std::fixed << std::setprecision(3)
                   << "trigger=" << state.right.trigger;

    return {
        "========== 外骨骼 -> TIAGo 遥操作状态 ==========",
        panelLine(
            "外骨骼",
            exoskeleton.stateFresh() ? "OK" : "ERROR",
            "connected | device=" + exoskeleton.device()),
        panelLine("模式", "OK", mode_detail.str()),
        panelLine(
            "左轮盘",
            "OK",
            "x/y=" + std::to_string(state.left.x) + "/" +
                std::to_string(state.left.y)),
        panelLine("TIAGo反馈", "OK", "Executor feedback active"),
        panelLine("左臂", "OK", left_arm_detail.str()),
        panelLine("右臂", "OK", right_arm_detail.str()),
        panelLine("左夹爪", "OK", left_gripper_detail),
        panelLine("右夹爪", "OK", gripper_detail.str()),
        panelLine("底盘", "OK", base_detail.str()),
        panelLine("总体", "OK", "RUNNING | Ctrl-C 停止")};
}

struct ReadySnapshot
{
    ExoskeletonState exoskeleton;
    Executor::RobotState robot;
    MappedTargets targets;
};

ReadySnapshot waitForReadiness(
    Exoskeleton &exoskeleton,
    const InitialFeedback &initial_feedback,
    const TeleopConfig &config,
    const robot::tiago::BaseConfig &base_config,
    Arm &left_arm,
    Arm &right_arm,
    Gripper &left_gripper,
    Gripper &right_gripper,
    robot::tiago::Base &base,
    const Arm::JointValues &left_arm_velocity_limits,
    const Arm::JointValues &right_arm_velocity_limits,
    const Gripper::FingerValues &left_gripper_velocity_limits,
    const Gripper::FingerValues &right_gripper_velocity_limits,
    StatusPanel &status_panel)
{
    const auto robot_state = makeRobotState(initial_feedback);
    std::string last_reason;
    auto last_panel_update = std::chrono::steady_clock::now() -
                             std::chrono::seconds{1};

    while (!stop_requested.load())
    {
        const auto state = exoskeleton.latestState();
        std::string reason;
        if (!state)
        {
            reason = "等待外骨骼首个完整合法帧";
        }
        else
        {
            reason = readinessReason(
                exoskeleton.stateFresh(),
                *state,
                robot_state,
                config,
                base_config,
                left_arm,
                right_arm,
                left_gripper,
                right_gripper,
                base,
                left_arm_velocity_limits,
                right_arm_velocity_limits,
                left_gripper_velocity_limits,
                right_gripper_velocity_limits);
        }

        const auto now = std::chrono::steady_clock::now();
        if (reason.empty())
        {
            status_panel.update(readinessPanelLines(
                exoskeleton,
                state,
                robot_state,
                config,
                reason));
            ReadySnapshot result{
                *state,
                robot_state,
                mapState(*state, config, base_config)};
            return result;
        }
        if (reason != last_reason ||
            now - last_panel_update >= std::chrono::milliseconds{100})
        {
            status_panel.update(readinessPanelLines(
                exoskeleton,
                state,
                robot_state,
                config,
                reason));
            last_reason = reason;
            last_panel_update = now;
        }

        std::this_thread::sleep_for(std::min(
            config.loop_period,
            std::chrono::milliseconds{20}));
    }

    throw std::runtime_error("收到停止请求，未启动 TIAGo 遥操作");
}

void sendSafeTargets(
    Executor &executor,
    const Gripper::FingerValues &left_gripper_velocity_limits,
    const Gripper::FingerValues &right_gripper_velocity_limits,
    bool &grippers_held)
{
    // BaseController 当前没有内建命令超时，因此每次安全周期都刷新零速。
    executor.setBaseVelocity(0.0, 0.0);

    if (grippers_held)
    {
        return;
    }

    // 夹爪没有 Arm 那样的 Servo timeout；断流时把目标改为最新反馈位置，
    // 让控制器停在当前位置，而不是继续执行旧的扳机目标。
    const auto state = executor.latestState();
    Gripper::FingerValues left_current{};
    Gripper::FingerValues right_current{};
    if (!copyCompletePositions(state.left_gripper_positions, left_current) ||
        !copyCompletePositions(state.right_gripper_positions, right_current))
    {
        throw std::runtime_error(
            "外骨骼不安全时无法取得完整夹爪反馈，退出并禁用 TIAGo 电机");
    }

    executor.setLeftGripperTarget(left_current, left_gripper_velocity_limits);
    executor.setRightGripperTarget(right_current, right_gripper_velocity_limits);
    grippers_held = true;
}

void sendTeleopTargets(
    Executor &executor,
    const MappedTargets &targets,
    const Arm::JointValues &left_arm_velocity_limits,
    const Arm::JointValues &right_arm_velocity_limits,
    const Gripper::FingerValues &left_gripper_velocity_limits,
    const Gripper::FingerValues &right_gripper_velocity_limits)
{
    executor.setLeftArmServoTarget(targets.left_arm, left_arm_velocity_limits);
    executor.setRightArmServoTarget(targets.right_arm, right_arm_velocity_limits);
    executor.setLeftGripperTarget(
        targets.left_gripper,
        left_gripper_velocity_limits);
    executor.setRightGripperTarget(
        targets.right_gripper,
        right_gripper_velocity_limits);
    executor.setBaseVelocity(
        targets.linear_velocity,
        targets.angular_velocity);
}

std::string modeHex(const std::uint8_t value)
{
    std::ostringstream output;
    output << "0x" << std::uppercase << std::hex << std::setw(2)
           << std::setfill('0') << static_cast<unsigned int>(value);
    return output.str();
}

} // namespace

int main(int argc, char **argv)
{
    try
    {
        const auto options = parseOptions(argc, argv);
        const auto exoskeleton_config =
            robot::input::exoskeleton::loadExoskeletonConfig(
                options.exoskeleton_config);
        const auto teleop_config = loadTeleopConfig(options.teleop_config);
        const auto tiago_configs = loadTiagoConfigs(options.tiago_root);

        std::cout << "外骨骼 VID:PID = 0x" << std::hex << std::setw(4)
                  << std::setfill('0') << exoskeleton_config.usb_vid
                  << ":0x" << std::setw(4) << exoskeleton_config.usb_pid
                  << std::dec << std::setfill(' ') << '\n'
                  << "TIAGo 配置根目录: " << options.tiago_root << '\n'
                  << "左轮盘: y -> "
                  << (teleop_config.linear_axis_sign > 0 ? "前" : "后")
                  << "/"
                  << (teleop_config.linear_axis_sign > 0 ? "后" : "前")
                  << ", x -> "
                  << (teleop_config.angular_axis_sign > 0 ? "左转" : "右转")
                  << "/"
                  << (teleop_config.angular_axis_sign > 0 ? "右转" : "左转")
                  << '\n'
                  << "左右扳机: 分别控制左右夹爪，0=张开，1=闭合\n"
                  << "模式: 运行=" << modeHex(teleop_config.run_mode_raw)
                  << ", 退出=" << modeHex(teleop_config.exit_mode_raw) << '\n';

        if (options.dry_run)
        {
            std::cout << "dry-run: 配置检查通过；未打开串口、SocketCAN 或任何电机。\n";
            return EXIT_SUCCESS;
        }
        if (!options.confirm)
        {
            std::cout << "未启动实体遥操作。若确认现场安全，请重新运行并显式添加 --confirm。\n";
            return EXIT_SUCCESS;
        }

        SingleProcessLock process_lock{teleop_config.lock_file};
        StatusPanel status_panel;
        Exoskeleton exoskeleton{exoskeleton_config};

        robot::tiago::Arm left_arm{
            tiago_configs.left_shoulder,
            tiago_configs.left_elbow,
            tiago_configs.left_wrist};
        robot::tiago::Arm right_arm{
            tiago_configs.right_shoulder,
            tiago_configs.right_elbow,
            tiago_configs.right_wrist};
        Gripper left_gripper{tiago_configs.left_gripper};
        Gripper right_gripper{tiago_configs.right_gripper};
        robot::tiago::Head head{tiago_configs.head};
        robot::tiago::Torso torso{tiago_configs.torso};
        robot::tiago::Base base{tiago_configs.base};

        robot::tiago::ArmController left_arm_controller{left_arm};
        robot::tiago::ArmController right_arm_controller{right_arm};
        robot::tiago::GripperController left_gripper_controller{left_gripper};
        robot::tiago::GripperController right_gripper_controller{right_gripper};
        robot::tiago::HeadController head_controller{head};
        robot::tiago::TorsoController torso_controller{torso};
        robot::tiago::BaseController base_controller{base};

        Executor::Config executor_config;
        executor_config.control_period =
            std::chrono::duration_cast<Executor::Duration>(
                teleop_config.executor_control_period);
        executor_config.command_timeout =
            std::chrono::duration_cast<Executor::Duration>(
                teleop_config.command_timeout);
        Executor executor{
            left_arm_controller,
            right_arm_controller,
            left_gripper_controller,
            right_gripper_controller,
            head_controller,
            torso_controller,
            base_controller,
            executor_config};
        HardwareGuard hardware_guard{
            executor,
            left_arm,
            right_arm,
            left_gripper,
            right_gripper,
            base};

        const auto left_arm_velocity_limits = makeArmVelocityLimits(
            tiago_configs.left_shoulder,
            tiago_configs.left_elbow,
            tiago_configs.left_wrist,
            teleop_config.arm_velocity_fraction);
        const auto right_arm_velocity_limits = makeArmVelocityLimits(
            tiago_configs.right_shoulder,
            tiago_configs.right_elbow,
            tiago_configs.right_wrist,
            teleop_config.arm_velocity_fraction);
        const auto left_gripper_velocity_limits = makeGripperVelocityLimits(
            tiago_configs.left_gripper,
            teleop_config.gripper_velocity_fraction);
        const auto right_gripper_velocity_limits = makeGripperVelocityLimits(
            tiago_configs.right_gripper,
            teleop_config.gripper_velocity_fraction);

        std::signal(SIGINT, signalHandler);
        std::signal(SIGTERM, signalHandler);

        // 外骨骼只读启动；TIAGo 只做主动状态查询，不发送运动目标。
        exoskeleton.start();

        InitialFeedback initial_feedback;
        waitForRobotFeedback(
            left_arm,
            right_arm,
            left_gripper,
            right_gripper,
            teleop_config.robot_feedback_timeout,
            initial_feedback);

        const auto ready = waitForReadiness(
            exoskeleton,
            initial_feedback,
            teleop_config,
            tiago_configs.base,
            left_arm,
            right_arm,
            left_gripper,
            right_gripper,
            base,
            left_arm_velocity_limits,
            right_arm_velocity_limits,
            left_gripper_velocity_limits,
            right_gripper_velocity_limits,
            status_panel);

        InitialFeedback ready_feedback;
        if (!extractInitialFeedback(ready.robot, ready_feedback))
        {
            throw std::runtime_error(
                "初始接管快照缺少双臂或双夹爪反馈");
        }

        if (options.clear_fault)
        {
            left_arm.clearFault();
            right_arm.clearFault();
            left_gripper.clearFault();
            right_gripper.clearFault();
            base.clearFault();
        }

        hardware_guard.markHardwareEnableStarted();
        left_arm.enable();
        right_arm.enable();
        left_gripper.enable();
        right_gripper.enable();
        base.enable();

        left_arm_controller.start(
            ready_feedback.left_arm,
            left_arm_velocity_limits);
        right_arm_controller.start(
            ready_feedback.right_arm,
            right_arm_velocity_limits);
        left_gripper_controller.start(
            ready_feedback.left_gripper,
            left_gripper_velocity_limits);
        right_gripper_controller.start(
            ready_feedback.right_gripper,
            right_gripper_velocity_limits);
        base_controller.start();

        executor.setLeftArmControlMode(Executor::ArmControlMode::Servo);
        executor.setRightArmControlMode(Executor::ArmControlMode::Servo);
        // 首个目标使用就绪检查时的目标；其误差受初始接管阈值限制，
        // 不会从一个未确认的外骨骼姿态突然跳转。
        sendTeleopTargets(
            executor,
            ready.targets,
            left_arm_velocity_limits,
            right_arm_velocity_limits,
            left_gripper_velocity_limits,
            right_gripper_velocity_limits);
        executor.start();
        hardware_guard.markExecutorRunning();

        status_panel.update(runningPanelLines(
            exoskeleton,
            ready.exoskeleton,
            ready.targets));

        bool needs_rearm = false;
        bool grippers_held = false;
        auto last_status_update = std::chrono::steady_clock::now() -
                                  std::chrono::seconds{1};
        while (!stop_requested.load())
        {
            if (executor.state() == Executor::State::Faulted)
            {
                throw std::runtime_error(
                    "TIAGo Executor 故障: " + executor.faultMessage());
            }

            const auto state = exoskeleton.latestState();
            const bool exit_mode =
                state &&
                state->left_mode_raw == teleop_config.exit_mode_raw &&
                state->right_mode_raw == teleop_config.exit_mode_raw;
            if (exit_mode)
            {
                sendSafeTargets(
                    executor,
                    left_gripper_velocity_limits,
                    right_gripper_velocity_limits,
                    grippers_held);
                status_panel.finish();
                std::cout << "检测到外骨骼退出档位，停止遥操作。\n"
                          << std::flush;
                break;
            }

            const bool safe_to_follow = state && exoskeleton.stateFresh() &&
                                         state->left_mode_raw ==
                                             teleop_config.run_mode_raw &&
                                         state->right_mode_raw ==
                                             teleop_config.run_mode_raw;
            if (!safe_to_follow)
            {
                needs_rearm = true;
                sendSafeTargets(
                    executor,
                    left_gripper_velocity_limits,
                    right_gripper_velocity_limits,
                    grippers_held);
                const auto now = std::chrono::steady_clock::now();
                if (now - last_status_update >=
                    std::chrono::milliseconds{100})
                {
                    const auto reason = state
                                             ? readinessReason(
                                                   exoskeleton.stateFresh(),
                                                   *state,
                                                   executor.latestState(),
                                                   teleop_config,
                                                   tiago_configs.base,
                                                   left_arm,
                                                   right_arm,
                                                   left_gripper,
                                                   right_gripper,
                                                   base,
                                                   left_arm_velocity_limits,
                                                   right_arm_velocity_limits,
                                                   left_gripper_velocity_limits,
                                                   right_gripper_velocity_limits)
                                             : "等待外骨骼首个完整合法帧";
                    status_panel.update(readinessPanelLines(
                        exoskeleton,
                        state,
                        executor.latestState(),
                        teleop_config,
                        reason));
                    last_status_update = now;
                }
                std::this_thread::sleep_for(teleop_config.loop_period);
                continue;
            }

            if (needs_rearm)
            {
                const auto reason = readinessReason(
                    true,
                    *state,
                    executor.latestState(),
                    teleop_config,
                    tiago_configs.base,
                    left_arm,
                    right_arm,
                    left_gripper,
                    right_gripper,
                    base,
                    left_arm_velocity_limits,
                    right_arm_velocity_limits,
                    left_gripper_velocity_limits,
                    right_gripper_velocity_limits);
                sendSafeTargets(
                    executor,
                    left_gripper_velocity_limits,
                    right_gripper_velocity_limits,
                    grippers_held);
                const auto now = std::chrono::steady_clock::now();
                if (now - last_status_update >=
                    std::chrono::milliseconds{100})
                {
                    status_panel.update(readinessPanelLines(
                        exoskeleton,
                        state,
                        executor.latestState(),
                        teleop_config,
                        reason));
                    last_status_update = now;
                }
                if (reason.empty())
                {
                    needs_rearm = false;
                    grippers_held = false;
                }
                std::this_thread::sleep_for(teleop_config.loop_period);
                continue;
            }

            const auto targets = mapState(
                *state,
                teleop_config,
                tiago_configs.base);
            validateMappedTargets(
                targets,
                left_arm,
                right_arm,
                left_gripper,
                right_gripper,
                base,
                left_arm_velocity_limits,
                right_arm_velocity_limits,
                left_gripper_velocity_limits,
                right_gripper_velocity_limits);
            sendTeleopTargets(
                executor,
                targets,
                left_arm_velocity_limits,
                right_arm_velocity_limits,
                left_gripper_velocity_limits,
                right_gripper_velocity_limits);
            grippers_held = false;

            const auto now = std::chrono::steady_clock::now();
            if (now - last_status_update >=
                std::chrono::milliseconds{100})
            {
                status_panel.update(runningPanelLines(
                    exoskeleton,
                    *state,
                    targets));
                last_status_update = now;
            }
            std::this_thread::sleep_for(teleop_config.loop_period);
        }

        hardware_guard.stopAndDisable();
        return EXIT_SUCCESS;
    }
    catch (const std::exception &error)
    {
        std::cerr << "外骨骼 TIAGo 遥操作未启动或已停止: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
