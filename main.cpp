#include "can/can_interface_manager.hpp"
#include "common/logger.hpp"

#include "ti5/can/can_bus.hpp"
#include "ti5/can/can_discovery.hpp"
#include "ti5/can/encoder_conversion.hpp"
#include "ti5/config/config_loader.hpp"
#include "ti5/motor/can_motor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace
{

using namespace std::chrono_literals;

constexpr std::uint16_t kTestNodeId = 17;
constexpr auto kControlPeriod = 10ms;

// 测试方向由符号决定：正值和负值分别测试相反方向。
// 当前使用现场确认的负方向，移动 0.10 rad 后返回起点。
constexpr double kOutwardDeltaRad = -0.10;
constexpr int kReadyProbeCycles = 30;
constexpr int kMoveCycles = 100;
constexpr int kHoldCycles = 30;
constexpr int kReturnHoldCycles = 50;
constexpr int kMaximumStaleCycles = 3;

// 来自当前 safety.yaml 的右肩 Roll 配置范围；尚未实机认证，因此还保留
// 0.02 rad 的边界余量，并且只允许本次极小相对位移。
constexpr double kConfiguredMinimumRad = -1.396263402;
constexpr double kConfiguredMaximumRad = 1.396263402;
constexpr double kLimitMarginRad = 0.02;
constexpr double kAllowedOvershootRad = 0.02;

class SingleProcessLock final
{
public:
    SingleProcessLock()
    {
        fd_ = ::open("/tmp/zk_robot_ti5_motion.lock",
                     O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW,
                     0600);
        if (fd_ < 0)
        {
            throw std::runtime_error("Cannot open TI5 motion lock");
        }
        if (::flock(fd_, LOCK_EX | LOCK_NB) != 0)
        {
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error("Another zk_robot motion test is running");
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
    std::error_code error;
    const std::filesystem::directory_iterator end;
    for (std::filesystem::directory_iterator entry{
             "/proc",
             std::filesystem::directory_options::skip_permission_denied,
             error};
         !error && entry != end;
         entry.increment(error))
    {
        const auto name = entry->path().filename().string();
        if (name.empty() ||
            !std::all_of(name.begin(), name.end(), [](const unsigned char value)
                         { return value >= '0' && value <= '9'; }))
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

void requireExplicitMotionConfirmation()
{
    const char *value = std::getenv("ZK_ROBOT_CONFIRM_RIGHT_ARM_TEST");
    if (value == nullptr || std::string{value} != "YES")
    {
        throw std::runtime_error(
            "Motion is locked. Set ZK_ROBOT_CONFIRM_RIGHT_ARM_TEST=YES after clearing the workspace");
    }
}

double quinticBlend(const double ratio)
{
    const double t = std::clamp(ratio, 0.0, 1.0);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

} // namespace

int main()
{
    try
    {
        using namespace robot;

        requireExplicitMotionConfirmation();
        if (::geteuid() == 0)
        {
            throw std::runtime_error(
                "Run this test as kuang, not with sudo/root");
        }
        SingleProcessLock process_lock;

        if (processNamedIsRunning("Ti5Control") ||
            processNamedIsRunning("joint_manager"))
        {
            throw std::runtime_error(
                "Ti5Control/joint_manager is running. Initialize if needed, close it, then run this test as the sole controller");
        }

        common::logger()->info(
            "TI5 R_SHOULDER_R node 17: outward 0.01 rad and return test");

        const auto robot_config = ti5::loadRobotConfig(
            std::filesystem::path{"config/ti5/t170c/robot.yaml"});
        const auto can_config = ti5::loadCanConfig(
            std::filesystem::path{"config/ti5/t170c/can.yaml"});
        const auto discovery_options = ti5::makeDiscoveryOptions(can_config);

        can::CanInterfaceManager interface_manager;
        const auto all_interfaces = interface_manager.enumerate(
            can_config.socketcan.interface_regex);
        if (all_interfaces.empty())
        {
            throw std::runtime_error("No SocketCAN interfaces found");
        }

        const auto &selector = can_config.socketcan.body_adapter;
        const auto body_interfaces = interface_manager.selectAdapter(
            all_interfaces,
            can::CanAdapterSelector{
                selector.selector,
                selector.value,
                selector.expected_channels});

        // 本测试绝不 down/up 或重配 CAN，避免影响机器人其他总线。
        std::vector<std::string> candidate_interfaces;
        for (const auto &interface : body_interfaces)
        {
            const auto ready = interface_manager.inspect(interface.name);
            if (!ready.up || !ready.bitrate ||
                *ready.bitrate != can_config.socketcan.bitrate)
            {
                throw std::runtime_error(
                    interface.name + " must already be UP at 1 Mbps");
            }
            candidate_interfaces.push_back(interface.name);
        }

        ti5::CanDiscovery discovery;
        const auto discovery_result = discovery.discover(
            robot_config.can_buses,
            discovery_options,
            candidate_interfaces);
        if (!discovery_result.success)
        {
            throw std::runtime_error("TI5 body CAN discovery failed");
        }

        const auto right_arm_bus = std::find_if(
            discovery_result.logical_buses.begin(),
            discovery_result.logical_buses.end(),
            [](const auto &bus)
            { return bus.bus_name == "right_arm"; });
        if (right_arm_bus == discovery_result.logical_buses.end() ||
            !right_arm_bus->complete ||
            !right_arm_bus->interface_name)
        {
            throw std::runtime_error("right_arm CAN bus not found");
        }

        const auto joint = std::find_if(
            robot_config.joints.begin(),
            robot_config.joints.end(),
            [](const auto &candidate)
            {
                return candidate.bus == "right_arm" &&
                       candidate.motor.node_id == kTestNodeId;
            });
        if (joint == robot_config.joints.end() ||
            joint->physical_name != "R_SHOULDER_R")
        {
            throw std::runtime_error("R_SHOULDER_R node 17 config not found");
        }

        common::logger()->info(
            "Test joint: {} / {} / node {} / {}",
            joint->name,
            joint->physical_name,
            joint->motor.node_id,
            *right_arm_bus->interface_name);

        ti5::CanBus bus{*right_arm_bus->interface_name};
        ti5::CanMotor motor{joint->motor, bus};

        const auto queried_position = motor.queryPosition();
        if (!queried_position)
        {
            throw std::runtime_error("Initial 0x08 position query failed");
        }

        const auto initial_csp = motor.queryCspStatus();
        if (!initial_csp)
        {
            throw std::runtime_error("Initial 0x41 CSP query failed");
        }

        const double q0 = ti5::positionCountsToRadians(
            initial_csp->position_counts,
            joint->motor.encoder.counts_per_output_revolution);
        if (std::abs(q0 - *queried_position) > 0.01)
        {
            throw std::runtime_error("0x08 and 0x41 positions disagree");
        }

        const double outward_target = q0 + kOutwardDeltaRad;
        if (q0 < kConfiguredMinimumRad + kLimitMarginRad ||
            q0 > kConfiguredMaximumRad - kLimitMarginRad ||
            outward_target < kConfiguredMinimumRad + kLimitMarginRad ||
            outward_target > kConfiguredMaximumRad - kLimitMarginRad)
        {
            throw std::runtime_error(
                "Current/target position is too close to the configured shoulder-roll limit");
        }

        common::logger()->info(
            "Start={:.6f} rad, outward target={:.6f} rad",
            q0,
            outward_target);

        auto state = motor.latestState();
        if (!state || !state->position_counts)
        {
            throw std::runtime_error("No initial MotorState");
        }
        std::uint64_t last_sequence = state->update_sequence;

        // 只用当前位置探测 0x44 readiness；没有稳定回传就不真正移动。
        auto next_cycle = std::chrono::steady_clock::now();
        int fresh_ready_feedback = 0;
        int stale_cycles = 0;
        for (int cycle = 0; cycle < kReadyProbeCycles; ++cycle)
        {
            next_cycle += kControlPeriod;
            motor.commandPositionCsp(q0);
            std::this_thread::sleep_until(next_cycle);

            state = motor.latestState();
            if (state && state->position_counts &&
                state->update_sequence > last_sequence)
            {
                last_sequence = state->update_sequence;
                ++fresh_ready_feedback;
                stale_cycles = 0;
            }
            else
            {
                ++stale_cycles;
            }
        }
        if (fresh_ready_feedback < kReadyProbeCycles / 2 ||
            stale_cycles >= kMaximumStaleCycles)
        {
            throw std::runtime_error(
                "CSP is not ready. No movement sent; initialize with Ti5Control, close it, and retry");
        }

        auto run_segment = [&](const char *name,
                               const double from,
                               const double to,
                               const int cycles)
        {
            common::logger()->info("Segment {}: {:.6f} -> {:.6f}",
                                   name,
                                   from,
                                   to);
            next_cycle = std::chrono::steady_clock::now();
            int local_stale_cycles = 0;

            for (int cycle = 1; cycle <= cycles; ++cycle)
            {
                const double blend = quinticBlend(
                    static_cast<double>(cycle) /
                    static_cast<double>(cycles));
                const double target = from + (to - from) * blend;

                next_cycle += kControlPeriod;
                motor.commandPositionCsp(target);
                std::this_thread::sleep_until(next_cycle);

                state = motor.latestState();
                if (!state || !state->position_counts ||
                    state->update_sequence <= last_sequence)
                {
                    ++local_stale_cycles;
                }
                else
                {
                    last_sequence = state->update_sequence;
                    local_stale_cycles = 0;
                }

                if (local_stale_cycles >= kMaximumStaleCycles)
                {
                    throw std::runtime_error(
                        std::string{"CSP feedback stale during "} + name);
                }

                if (state && state->position_counts)
                {
                    const double measured = ti5::positionCountsToRadians(
                        state->position_counts->value,
                        joint->motor.encoder.counts_per_output_revolution);
                    if (std::abs(measured - q0) >
                        std::abs(kOutwardDeltaRad) + kAllowedOvershootRad)
                    {
                        throw std::runtime_error(
                            "Observed shoulder movement exceeded commanded delta plus 0.02 rad");
                    }
                    if (cycle % 20 == 0 || cycle == cycles)
                    {
                        common::logger()->info(
                            "{} {}/{}: target={:.6f}, measured={:.6f}, seq={}",
                            name,
                            cycle,
                            cycles,
                            target,
                            measured,
                            state->update_sequence);
                    }
                }
            }
        };

        run_segment("test-direction", q0, outward_target, kMoveCycles);
        run_segment("test-hold", outward_target, outward_target, kHoldCycles);

        state = motor.latestState();
        if (!state || !state->position_counts)
        {
            throw std::runtime_error("No feedback at outward endpoint");
        }
        const double outward_measured = ti5::positionCountsToRadians(
            state->position_counts->value,
            joint->motor.encoder.counts_per_output_revolution);
        run_segment("return", outward_target, q0, kMoveCycles);
        run_segment("return-hold", q0, q0, kReturnHoldCycles);

        state = motor.latestState();
        if (!state || !state->position_counts)
        {
            throw std::runtime_error("No final position feedback");
        }
        const double final_position = ti5::positionCountsToRadians(
            state->position_counts->value,
            joint->motor.encoder.counts_per_output_revolution);
        if (std::abs(final_position - q0) > 0.003)
        {
            throw std::runtime_error("Right shoulder did not return to its start position");
        }

        common::logger()->info(
            "PASS: start={:.6f}, test-endpoint={:.6f}, final={:.6f} rad",
            q0,
            outward_measured,
            final_position);
        common::logger()->info(
            "No 0x01 and no automatic STOP/disable were sent");
        return 0;
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error("Test failed: {}", error.what());
        return 1;
    }
}
