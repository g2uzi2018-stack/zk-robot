#include "tiago/arm/arm.hpp"
#include "tiago/can/can_config.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <thread>

int main()
{
    try
    {
        // ------------------------------------------------------------
        // 1. 加载左臂三条 CAN 总线配置。
        // ------------------------------------------------------------
        const auto shoulder_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_shoulder.yaml");

        const auto elbow_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_elbow.yaml");

        const auto wrist_config = robot::tiago::loadCanBusConfig("config/tiago/can/left_wrist.yaml");

        // ------------------------------------------------------------
        // 2. 创建完整左臂。
        // ------------------------------------------------------------
        robot::tiago::Arm arm(shoulder_config, elbow_config, wrist_config);

        // ------------------------------------------------------------
        // 3. 设置 7 个关节目标位置和速度限制。
        // ------------------------------------------------------------
        const robot::tiago::Arm::JointValues target_positions{
            0.2, 0.3, 0.4, 0.5, 0.2, 0.3, 0.4};

        const robot::tiago::Arm::JointValues velocity_limits{
            0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

        std::cout << "Left arm joints\n";

        for (std::size_t i = 0; i < robot::tiago::Arm::kJointCount; ++i)
        {
            std::cout << "  joint " << (i + 1) << ": " << arm.joint(i).name() << '\n';
        }

        // ------------------------------------------------------------
        // 4. 使能整条机械臂。
        // ------------------------------------------------------------
        std::cout << "\nEnable arm\n";
        arm.enable();

        std::cout << "\nTargets\n";

        for (std::size_t i = 0; i < robot::tiago::Arm::kJointCount; ++i)
        {
            std::cout << "  joint " << (i + 1) << ": " << target_positions[i]
                      << " rad, velocity " << velocity_limits[i] << " rad/s\n";
        }

        // ------------------------------------------------------------
        // 5. 模拟 Controller，以 100 ms 周期刷新整臂目标。
        // ------------------------------------------------------------
        constexpr auto kControlPeriod = std::chrono::milliseconds{100};

        constexpr int kMaximumCycles = 80;

        for (int cycle = 1; cycle <= kMaximumCycles; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            // 一次发送完整 7 Joint 目标。
            arm.commandPositions(target_positions, velocity_limits);

            // 每 10 个周期读取一次反馈。
            if (cycle % 10 == 0)
            {
                const auto positions = arm.readPositions();

                std::cout << "\n[cycle " << cycle << "]\n";

                for (std::size_t i = 0; i < robot::tiago::Arm::kJointCount; ++i)
                {
                    std::cout << "  joint " << (i + 1) << " position: ";

                    if (positions[i])
                    {
                        std::cout << *positions[i] << " rad\n";
                    }
                    else
                    {
                        std::cout << "no feedback\n";
                    }
                }
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        // ------------------------------------------------------------
        // 6. 最后读取一次整臂位置。
        // ------------------------------------------------------------
        const auto final_positions = arm.readPositions();

        std::cout << "\nFinal positions\n";

        for (std::size_t i = 0; i < robot::tiago::Arm::kJointCount; ++i)
        {
            std::cout << "  joint " << (i + 1) << ": ";

            if (final_positions[i])
            {
                std::cout << *final_positions[i] << " rad";
            }
            else
            {
                std::cout << "no feedback";
            }

            std::cout << "  target: " << target_positions[i] << " rad\n";
        }

        // ------------------------------------------------------------
        // 7. 禁用整条机械臂。
        // ------------------------------------------------------------
        std::cout << "\nDisable arm\n";
        arm.disable();

        std::cout << "Arm control test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
