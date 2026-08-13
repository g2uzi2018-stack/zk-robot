#include "tiago/arm/arm.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/controller/arm_controller.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
    const char *stateName(
        robot::tiago::ArmController::ControlState state)
    {
        using State =
            robot::tiago::ArmController::ControlState;

        switch (state)
        {
        case State::Idle:
            return "Idle";

        case State::Running:
            return "Running";

        case State::Failed:
            return "Failed";
        }

        return "Unknown";
    }

    void printTarget(
        const robot::tiago::Arm::JointValues &target)
    {
        for (std::size_t i = 0;
             i < robot::tiago::Arm::kJointCount;
             ++i)
        {
            std::cout
                << "  joint "
                << (i + 1)
                << ": "
                << target[i]
                << '\n';
        }
    }

    void printPositions(
        const robot::tiago::Arm::JointPositions &positions)
    {
        for (std::size_t i = 0;
             i < robot::tiago::Arm::kJointCount;
             ++i)
        {
            std::cout
                << "  joint "
                << (i + 1)
                << ": ";

            if (positions[i])
            {
                std::cout
                    << *positions[i]
                    << " rad\n";
            }
            else
            {
                std::cout
                    << "no feedback\n";
            }
        }
    }
}

int main()
{
    try
    {
        // ------------------------------------------------------------
        // 1. 创建 Arm 和 Controller。
        // ------------------------------------------------------------
        const auto shoulder_config =
            robot::tiago::loadCanBusConfig(
                "config/tiago/can/left_shoulder.yaml");

        const auto elbow_config =
            robot::tiago::loadCanBusConfig(
                "config/tiago/can/left_elbow.yaml");

        const auto wrist_config =
            robot::tiago::loadCanBusConfig(
                "config/tiago/can/left_wrist.yaml");

        robot::tiago::Arm arm(
            shoulder_config,
            elbow_config,
            wrist_config);

        robot::tiago::ArmController controller(arm);

        const robot::tiago::Arm::JointValues valid_target{
            0.2,
            0.3,
            0.4,
            0.5,
            0.2,
            0.3,
            0.4};

        const robot::tiago::Arm::JointValues velocity_limits{
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1,
            0.1};

        std::cout
            << "Initial state: "
            << stateName(controller.state())
            << "\n\n";

        // ------------------------------------------------------------
        // 2. Idle 状态调用 setTarget()。
        // ------------------------------------------------------------
        std::cout << "[TEST 1] setTarget() while Idle\n";

        try
        {
            controller.setTarget(
                valid_target,
                velocity_limits);

            std::cout << "FAIL: setTarget() was accepted\n";
        }
        catch (const std::logic_error &error)
        {
            std::cout
                << "PASS: "
                << error.what()
                << '\n';
        }

        // ------------------------------------------------------------
        // 3. Idle 状态调用 stop()。
        // ------------------------------------------------------------
        std::cout << "\n[TEST 2] stop() while Idle\n";

        try
        {
            controller.stop();

            std::cout << "FAIL: stop() was accepted\n";
        }
        catch (const std::logic_error &error)
        {
            std::cout
                << "PASS: "
                << error.what()
                << '\n';
        }

        // ------------------------------------------------------------
        // 4. Idle 状态调用 reset()。
        // ------------------------------------------------------------
        std::cout << "\n[TEST 3] reset() while Idle\n";

        try
        {
            controller.reset();

            std::cout << "FAIL: reset() was accepted\n";
        }
        catch (const std::logic_error &error)
        {
            std::cout
                << "PASS: "
                << error.what()
                << '\n';
        }

        // ------------------------------------------------------------
        // 5. 启动正常控制。
        // ------------------------------------------------------------
        std::cout << "\nEnable arm\n";
        arm.enable();

        controller.start(
            valid_target,
            velocity_limits);

        std::cout
            << "State after start: "
            << stateName(controller.state())
            << '\n';

        // ------------------------------------------------------------
        // 6. Running 状态调用 reset()。
        // ------------------------------------------------------------
        std::cout << "\n[TEST 4] reset() while Running\n";

        try
        {
            controller.reset();

            std::cout << "FAIL: reset() was accepted\n";
        }
        catch (const std::logic_error &error)
        {
            std::cout
                << "PASS: "
                << error.what()
                << '\n';
        }

        std::cout
            << "State: "
            << stateName(controller.state())
            << '\n';

        // ------------------------------------------------------------
        // 7. 先正常执行原目标一段时间。
        // ------------------------------------------------------------
        constexpr auto kControlPeriod =
            std::chrono::milliseconds{100};

        for (int cycle = 0; cycle < 10; ++cycle)
        {
            const auto cycle_start =
                std::chrono::steady_clock::now();

            controller.update();

            std::this_thread::sleep_until(
                cycle_start + kControlPeriod);
        }

        std::cout << "\nCurrent valid target\n";
        printTarget(controller.targetPositions());

        std::cout << "\nPositions before invalid setTarget\n";
        printPositions(controller.currentPositions());

        // ------------------------------------------------------------
        // 8. Running 状态发送非法新目标。
        //
        // joint 7 = 10 rad，故意超过机械限位。
        // ------------------------------------------------------------
        const robot::tiago::Arm::JointValues invalid_target{
            0.6,
            0.7,
            0.8,
            0.9,
            0.6,
            0.7,
            10.0};

        const auto target_before_invalid =
            controller.targetPositions();

        std::cout
            << "\n[TEST 5] invalid setTarget() while Running\n";

        try
        {
            controller.setTarget(
                invalid_target,
                velocity_limits);

            std::cout
                << "FAIL: invalid target was accepted\n";
        }
        catch (const std::exception &error)
        {
            std::cout
                << "PASS: caught expected exception: "
                << error.what()
                << '\n';
        }

        // ------------------------------------------------------------
        // 9. 检查 Controller 状态。
        // ------------------------------------------------------------
        std::cout
            << "\nController state after invalid target: "
            << stateName(controller.state())
            << '\n';

        if (controller.state() ==
            robot::tiago::ArmController::ControlState::Running)
        {
            std::cout << "PASS: controller is still Running\n";
        }
        else
        {
            std::cout << "FAIL: controller state changed\n";
        }

        // ------------------------------------------------------------
        // 10. 检查原来的 target 有没有被破坏。
        // ------------------------------------------------------------
        if (controller.targetPositions() ==
            target_before_invalid)
        {
            std::cout
                << "PASS: previous target was preserved\n";
        }
        else
        {
            std::cout
                << "FAIL: previous target was modified\n";
        }

        std::cout << "\nTarget after invalid setTarget\n";
        printTarget(controller.targetPositions());

        // ------------------------------------------------------------
        // 11. 再运行 10 个周期。
        //
        // Controller 应继续追原来的合法目标，
        // 而不是非法的新目标。
        // ------------------------------------------------------------
        std::cout
            << "\nContinue running previous valid target\n";

        for (int cycle = 1; cycle <= 10; ++cycle)
        {
            const auto cycle_start =
                std::chrono::steady_clock::now();

            controller.update();

            if (cycle == 10)
            {
                std::cout << "\nPositions after 10 more cycles\n";
                printPositions(
                    controller.currentPositions());
            }

            std::this_thread::sleep_until(
                cycle_start + kControlPeriod);
        }

        std::cout
            << "\nFinal controller state: "
            << stateName(controller.state())
            << '\n';

        // ------------------------------------------------------------
        // 12. 正常停止。
        // ------------------------------------------------------------
        std::cout << "\nStop controller\n";

        controller.stop();

        std::cout
            << "State after stop: "
            << stateName(controller.state())
            << '\n';

        arm.disable();

        std::cout
            << "\nArmController boundary test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Unexpected error: "
            << error.what()
            << '\n';

        return 1;
    }

    return 0;
}