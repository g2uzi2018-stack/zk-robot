#include "tiago/arm/arm.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/controller/arm_controller.hpp"
#include "tiago/gripper/gripper.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>

namespace
{
    // 机械臂和夹爪示例共用的控制周期。
    constexpr auto kControlPeriod = std::chrono::milliseconds{100};

    void printGripperPositions(const robot::tiago::Gripper::FingerPositions &positions)
    {
        const char *names[] = {"right finger", "left finger"};

        for (std::size_t i = 0; i < robot::tiago::Gripper::kFingerCount; ++i)
        {
            std::cout << "  " << names[i] << ": ";

            if (positions[i])
            {
                std::cout << *positions[i] << " m\n";
            }
            else
            {
                std::cout << "no feedback\n";
            }
        }
    }

    // 在夹爪运动期间持续刷新机械臂和夹爪控制目标。
    void runGripperTarget(robot::tiago::ArmController &arm_controller, robot::tiago::Gripper &gripper, double finger_position, double velocity_limit, int cycles)
    {
        std::cout << "\nGripper target: " << finger_position << " m\n";

        for (int cycle = 1; cycle <= cycles; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            // 持续维持机械臂姿态。
            arm_controller.update();

            // 持续刷新夹爪目标。
            gripper.commandSymmetric(finger_position, velocity_limit);

            const auto positions = gripper.readPositions();

            if (cycle == 1 || cycle % 5 == 0 || cycle == cycles)
            {
                std::cout << "\n[cycle " << cycle << "]\n";

                printGripperPositions(positions);
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }
    }
}

int main()
{
    try
    {
        // ------------------------------------------------------------
        // 1. 创建左臂。
        // ------------------------------------------------------------
        const auto shoulder_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_shoulder.yaml");
        const auto elbow_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_elbow.yaml");
        const auto wrist_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_wrist.yaml");
        robot::tiago::Arm arm(shoulder_config, elbow_config, wrist_config);

        robot::tiago::ArmController arm_controller(arm);

        // ------------------------------------------------------------
        // 2. 创建左夹爪。
        // ------------------------------------------------------------
        const auto gripper_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_gripper.yaml");
        robot::tiago::Gripper gripper(gripper_config);

        // ------------------------------------------------------------
        // 3. 左臂抬起姿态。
        // ------------------------------------------------------------
        const robot::tiago::Arm::JointValues arm_target{
            0.35,
            -0.60,
            1.20,
            1.00,
            -0.40,
            0.60,
            0.20};

        const robot::tiago::Arm::JointValues arm_velocity_limits{
            0.10,
            0.10,
            0.10,
            0.10,
            0.10,
            0.10,
            0.10};

        std::cout << "Clear arm faults\n";
        arm.clearFault();

        std::cout << "Enable arm\n";
        arm.enable();

        arm_controller.start(arm_target, arm_velocity_limits);

        // ------------------------------------------------------------
        // 4. 先只移动机械臂。
        // ------------------------------------------------------------
        std::cout << "\nRaise left arm\n";

        for (int cycle = 1; cycle <= 100; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            arm_controller.update();

            if (cycle % 10 == 0)
            {
                std::cout << "Arm cycle " << cycle << ", target reached: "
                          << (arm_controller.targetReached(0.01) ? "yes" : "no") << '\n';
            }

            if (arm_controller.targetReached(0.01))
            {
                std::cout << "Arm reached raised pose\n";
                break;
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        // ------------------------------------------------------------
        // 5. 机械臂保持运行，再启动夹爪。
        // ------------------------------------------------------------
        std::cout << "\nClear gripper faults\n";
        gripper.clearFault();

        std::cout << "Enable gripper\n";
        gripper.enable();

        std::this_thread::sleep_for(std::chrono::milliseconds{100});

        std::cout << "\nInitial gripper positions\n";

        printGripperPositions(gripper.readPositions());

        // ------------------------------------------------------------
        // 6. 打开夹爪。
        // ------------------------------------------------------------
        runGripperTarget(arm_controller, gripper, 0.025, 0.01, 30);

        // ------------------------------------------------------------
        // 7. 再收回。
        // ------------------------------------------------------------
        runGripperTarget(arm_controller, gripper, 0.010, 0.01, 30);

        // ------------------------------------------------------------
        // 8. 停止。
        // ------------------------------------------------------------
        std::cout << "\nStop gripper\n";
        gripper.stop();

        std::cout << "Disable gripper\n";
        gripper.disable();

        std::cout << "Stop arm controller\n";
        arm_controller.stop();

        std::cout << "Disable arm\n";
        arm.disable();

        std::cout << "\nArm + gripper test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
