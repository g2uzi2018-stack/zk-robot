#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/joint/joint.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

int main()
{
    try
    {
        // ------------------------------------------------------------
        // 1. 加载左肩 CAN 配置。
        // ------------------------------------------------------------
        const auto config = robot::tiago::loadCanBusConfig(
            "config/tiago/can/left_shoulder.yaml");

        if (config.joints.size() < 2)
        {
            throw std::runtime_error("left_shoulder config requires at least two joints");
        }

        const auto &joint1_config = config.joints[0];
        const auto &joint2_config = config.joints[1];

        std::cout << "CAN interface: " << config.interface_name << "\n\n";
        std::cout << "Joint 1: " << joint1_config.name << '\n';
        std::cout << "Joint 1 node ID: " << joint1_config.motor.node_id << "\n\n";
        std::cout << "Joint 2: " << joint2_config.name << '\n';
        std::cout << "Joint 2 node ID: " << joint2_config.motor.node_id << "\n\n";

        // ------------------------------------------------------------
        // 2. 创建唯一的一条共享 CAN 总线。
        // ------------------------------------------------------------
        robot::tiago::CanBus bus(config.interface_name);

        // ------------------------------------------------------------
        // 3. 两个 Joint 共享同一个 CanBus。
        // ------------------------------------------------------------
        robot::tiago::Joint joint1(joint1_config, bus);
        robot::tiago::Joint joint2(joint2_config, bus);

        // ------------------------------------------------------------
        // 4. 使能两个关节。
        // ------------------------------------------------------------
        std::cout << "Enable joint 1\n";
        joint1.enable();

        std::cout << "Enable joint 2\n";
        joint2.enable();

        // ------------------------------------------------------------
        // 5. 设置两个不同目标。
        // ------------------------------------------------------------
        constexpr double kJoint1Target = 0.3;
        constexpr double kJoint2Target = 0.7;
        constexpr double kVelocity = 0.1;

        std::cout << "\nJoint 1 target: " << kJoint1Target << " rad\n";
        std::cout << "Joint 2 target: " << kJoint2Target << " rad\n";
        std::cout << "Velocity: " << kVelocity << " rad/s\n\n";

        // ------------------------------------------------------------
        // 6. 模拟 Controller，100 ms 周期刷新两个 Joint。
        // ------------------------------------------------------------
        constexpr auto kControlPeriod = std::chrono::milliseconds{100};
        constexpr int kMaximumCycles = 80;

        for (int cycle = 1; cycle <= kMaximumCycles; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();
            // 两个关节都刷新目标。
            joint1.commandPosition(kJoint1Target, kVelocity);
            joint2.commandPosition(kJoint2Target, kVelocity);

            // 每 10 个周期打印一次位置，
            // 避免终端输出太密。
            if (cycle % 10 == 0)
            {
                const auto position1 = joint1.readPosition();
                const auto position2 = joint2.readPosition();

                std::cout << "[cycle " << cycle << "]\n";

                if (position1)
                {
                    std::cout << "  joint 1 position: " << *position1 << " rad\n";
                }
                else
                {
                    std::cout << "  joint 1 position: no feedback\n";
                }

                if (position2)
                {
                    std::cout << "  joint 2 position: " << *position2 << " rad\n";
                }
                else
                {
                    std::cout << "  joint 2 position: no feedback\n";
                }
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        // ------------------------------------------------------------
        // 7. 最后读取一次位置。
        // ------------------------------------------------------------
        std::cout << "\nFinal positions\n";
        const auto final_position1 = joint1.readPosition();
        const auto final_position2 = joint2.readPosition();

        if (final_position1)
        {
            std::cout << "Joint 1: " << *final_position1 << " rad\n";
        }
        else
        {
            std::cout << "Joint 1: no feedback\n";
        }

        if (final_position2)
        {
            std::cout << "Joint 2: " << *final_position2 << " rad\n";
        }
        else
        {
            std::cout << "Joint 2: no feedback\n";
        }

        // ------------------------------------------------------------
        // 8. 禁用两个关节。
        // ------------------------------------------------------------
        std::cout << "\nDisable joints\n";
        joint1.disable();
        joint2.disable();

        std::cout << "Dual joint test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
