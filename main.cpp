#include "common/logger.hpp"

#include "tiago/arm/arm.hpp"
#include "tiago/base/base.hpp"
#include "tiago/base/base_config.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/gripper/gripper.hpp"
#include "tiago/head/head.hpp"
#include "tiago/torso/torso.hpp"

#include "tiago/controller/arm_controller.hpp"
#include "tiago/controller/base_controller.hpp"
#include "tiago/controller/gripper_controller.hpp"
#include "tiago/controller/head_controller.hpp"
#include "tiago/controller/torso_controller.hpp"

#include "tiago/executor/robot_control_executor.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
    using namespace robot::tiago;

    constexpr auto kFeedbackTimeout = std::chrono::seconds{2};
    constexpr auto kFeedbackPollPeriod = std::chrono::milliseconds{10};

    Arm::JointValues waitArmPositions(Arm &arm, const char *name)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = arm.readPositions();
            Arm::JointValues values{};
            bool complete = true;

            for (std::size_t i = 0; i < Arm::kJointCount; ++i)
            {
                if (!positions[i])
                {
                    complete = false;
                    break;
                }

                values[i] = *positions[i];
            }

            if (complete)
            {
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error(std::string("Timed out waiting for ") + name + " feedback");
    }

    Gripper::FingerValues waitGripperPositions(Gripper &gripper, const char *name)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = gripper.readPositions();
            Gripper::FingerValues values{};

            if (positions[0] && positions[1])
            {
                values[0] = *positions[0];
                values[1] = *positions[1];
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error(std::string("Timed out waiting for ") + name + " feedback");
    }

    Head::JointValues waitHeadPositions(Head &head)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto positions = head.readPositions();
            Head::JointValues values{};

            if (positions[0] && positions[1])
            {
                values[0] = *positions[0];
                values[1] = *positions[1];
                return values;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error("Timed out waiting for head feedback");
    }

    double waitTorsoPosition(Torso &torso)
    {
        const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;

        while (std::chrono::steady_clock::now() < deadline)
        {
            const auto position = torso.readPosition();

            if (position)
            {
                return *position;
            }

            std::this_thread::sleep_for(kFeedbackPollPeriod);
        }

        throw std::runtime_error("Timed out waiting for torso feedback");
    }

    Arm::JointValues armVelocityLimits(
        const CanBusConfig &shoulder,
        const CanBusConfig &elbow,
        const CanBusConfig &wrist)
    {
        if (shoulder.joints.size() != 2 || elbow.joints.size() != 2 || wrist.joints.size() != 3)
        {
            throw std::logic_error("Invalid arm config while building velocity limits");
        }

        return {
            shoulder.joints[0].limits.max_velocity,
            shoulder.joints[1].limits.max_velocity,
            elbow.joints[0].limits.max_velocity,
            elbow.joints[1].limits.max_velocity,
            wrist.joints[0].limits.max_velocity,
            wrist.joints[1].limits.max_velocity,
            wrist.joints[2].limits.max_velocity};
    }

    Gripper::FingerValues gripperVelocityLimits(const CanBusConfig &config)
    {
        if (config.joints.size() != Gripper::kFingerCount)
        {
            throw std::logic_error("Invalid gripper config while building velocity limits");
        }

        return {
            config.joints[0].limits.max_velocity,
            config.joints[1].limits.max_velocity};
    }

    Head::JointValues headVelocityLimits(const CanBusConfig &config)
    {
        if (config.joints.size() != Head::kJointCount)
        {
            throw std::logic_error("Invalid head config while building velocity limits");
        }

        return {
            config.joints[0].limits.max_velocity,
            config.joints[1].limits.max_velocity};
    }
}

int main()
{
    using namespace robot::tiago;

    try
    {
        robot::common::logger()->info("Robot startup");

        // 加载全部机器人配置。
        const auto left_shoulder_config = loadCanBusConfig("config/tiago/can/left_shoulder.yaml");
        const auto left_elbow_config = loadCanBusConfig("config/tiago/can/left_elbow.yaml");
        const auto left_wrist_config = loadCanBusConfig("config/tiago/can/left_wrist.yaml");

        const auto right_shoulder_config = loadCanBusConfig("config/tiago/can/right_shoulder.yaml");
        const auto right_elbow_config = loadCanBusConfig("config/tiago/can/right_elbow.yaml");
        const auto right_wrist_config = loadCanBusConfig("config/tiago/can/right_wrist.yaml");

        const auto left_gripper_config = loadCanBusConfig("config/tiago/can/left_gripper.yaml");
        const auto right_gripper_config = loadCanBusConfig("config/tiago/can/right_gripper.yaml");
        const auto head_config = loadCanBusConfig("config/tiago/can/head.yaml");
        const auto torso_config = loadCanBusConfig("config/tiago/can/torso.yaml");
        const auto base_config = loadBaseConfig("config/tiago/base/base.yaml");

        // 创建物理组件。
        Arm left_arm(left_shoulder_config, left_elbow_config, left_wrist_config);
        Arm right_arm(right_shoulder_config, right_elbow_config, right_wrist_config);

        Gripper left_gripper(left_gripper_config);
        Gripper right_gripper(right_gripper_config);

        Head head(head_config);
        Torso torso(torso_config);
        Base base(base_config);

        // 创建 Controller。
        ArmController left_arm_controller(left_arm);
        ArmController right_arm_controller(right_arm);

        GripperController left_gripper_controller(left_gripper);
        GripperController right_gripper_controller(right_gripper);

        HeadController head_controller(head);
        TorsoController torso_controller(torso);
        BaseController base_controller(base);

        // 清理阶段采用 best-effort，避免一次 cleanup 异常阻断其他组件。
        const auto shutdownHardware = [&]() noexcept
        {
            const auto safe = [](const char *name, auto &&action) noexcept
            {
                try
                {
                    action();
                }
                catch (const std::exception &error)
                {
                    robot::common::logger()->warn("Cleanup {} failed: {}", name, error.what());
                }
                catch (...)
                {
                    robot::common::logger()->warn("Cleanup {} failed with unknown error", name);
                }
            };

            safe("left arm stop", [&] { left_arm.stop(); });
            safe("right arm stop", [&] { right_arm.stop(); });
            safe("left gripper stop", [&] { left_gripper.stop(); });
            safe("right gripper stop", [&] { right_gripper.stop(); });
            safe("head stop", [&] { head.stop(); });
            safe("torso stop", [&] { torso.stop(); });
            safe("base stop", [&] { base.stop(); });

            safe("left arm disable", [&] { left_arm.disable(); });
            safe("right arm disable", [&] { right_arm.disable(); });
            safe("left gripper disable", [&] { left_gripper.disable(); });
            safe("right gripper disable", [&] { right_gripper.disable(); });
            safe("head disable", [&] { head.disable(); });
            safe("torso disable", [&] { torso.disable(); });
            safe("base disable", [&] { base.disable(); });
        };

        try
        {
            // 清除故障并使能全部硬件。
            left_arm.clearFault();
            right_arm.clearFault();
            left_gripper.clearFault();
            right_gripper.clearFault();
            head.clearFault();
            torso.clearFault();
            base.clearFault();

            left_arm.enable();
            right_arm.enable();
            left_gripper.enable();
            right_gripper.enable();
            head.enable();
            torso.enable();
            base.enable();

            // 不假设启动姿态。
            // 等待 Webots 周期反馈，直接读取当前真实位置。
            robot::common::logger()->info("Waiting for current robot feedback");

            const auto left_arm_position = waitArmPositions(left_arm, "left arm");
            const auto right_arm_position = waitArmPositions(right_arm, "right arm");
            const auto left_gripper_position = waitGripperPositions(left_gripper, "left gripper");
            const auto right_gripper_position = waitGripperPositions(right_gripper, "right gripper");
            const auto head_position = waitHeadPositions(head);
            const double torso_position = waitTorsoPosition(torso);

            robot::common::logger()->info("Current robot feedback acquired");

            // 速度限制全部直接来自 YAML。
            const auto left_arm_velocity = armVelocityLimits(
                left_shoulder_config,
                left_elbow_config,
                left_wrist_config);

            const auto right_arm_velocity = armVelocityLimits(
                right_shoulder_config,
                right_elbow_config,
                right_wrist_config);

            const auto left_gripper_velocity = gripperVelocityLimits(left_gripper_config);
            const auto right_gripper_velocity = gripperVelocityLimits(right_gripper_config);
            const auto head_velocity = headVelocityLimits(head_config);
            const double torso_velocity = torso_config.joints.at(0).limits.max_velocity;

            // 以真实当前位置作为初始控制目标。
            // Executor 启动后只会保持当前姿态，不会主动跳到预设零位。
            left_arm_controller.start(left_arm_position, left_arm_velocity);
            right_arm_controller.start(right_arm_position, right_arm_velocity);

            left_gripper_controller.start(left_gripper_position, left_gripper_velocity);
            right_gripper_controller.start(right_gripper_position, right_gripper_velocity);

            head_controller.start(head_position, head_velocity);
            torso_controller.start(torso_position, torso_velocity);
            base_controller.start();

            RobotControlExecutor::Config executor_config;
            executor_config.control_period = std::chrono::milliseconds{100};
            executor_config.command_timeout = std::chrono::milliseconds{500};

            {
                RobotControlExecutor executor(
                    left_arm_controller,
                    right_arm_controller,
                    left_gripper_controller,
                    right_gripper_controller,
                    head_controller,
                    torso_controller,
                    base_controller,
                    executor_config);

                executor.start();

                robot::common::logger()->info(
                    "Executor integration test running for 5 seconds");

                std::this_thread::sleep_for(std::chrono::seconds{5});

                if (executor.state() == RobotControlExecutor::State::Faulted)
                {
                    throw std::runtime_error(
                        std::string("Executor fault: ") + executor.faultMessage());
                }

                const auto statistics = executor.statistics();
                const auto last_execution_us =
                    std::chrono::duration_cast<std::chrono::microseconds>(
                        statistics.last_execution_time)
                        .count();

                std::cout << "\n====================================\n"
                          << "EXECUTOR INTEGRATION TEST\n"
                          << "====================================\n"
                          << "Cycle count: " << statistics.cycle_count << '\n'
                          << "Last execution: " << last_execution_us << " us\n"
                          << "Deadline miss: " << statistics.deadline_miss_count << '\n'
                          << "====================================\n";
            }

            // Executor 已经退出，执行硬件清理。
            shutdownHardware();

            robot::common::logger()->info("Robot shutdown complete");
            return 0;
        }
        catch (...)
        {
            shutdownHardware();
            throw;
        }
    }
    catch (const std::exception &error)
    {
        robot::common::logger()->error(
            "Robot test failed: {}",
            error.what());

        return 1;
    }
}