#include "tiago/can/can_config.hpp"
#include "tiago/gripper/gripper.hpp"

#include <chrono>
#include <cstddef>
#include <iostream>
#include <thread>

namespace
{
    void printPositions(const robot::tiago::Gripper::FingerPositions &positions)
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
}

int main()
{
    try
    {
        // 只读取反馈，不发送任何会改变设备状态的命令。
        const auto config = robot::tiago::loadCanBusConfig("config/tiago/can/right_gripper.yaml");

        robot::tiago::Gripper gripper(config);

        std::cout << "Right gripper clean-start feedback test\n\n";

        std::cout << "CAN interface: " << config.interface_name << '\n';

        std::cout << "Finger 1: " << gripper.finger(0).name() << '\n';

        std::cout << "Finger 2: " << gripper.finger(1).name() << "\n\n";

        std::cout << "IMPORTANT:\n"
                  << "No clearFault\n"
                  << "No enable\n"
                  << "No position command\n"
                  << "No stop command\n\n";

        // 给 gateway 一点时间产生周期反馈。
        std::this_thread::sleep_for(std::chrono::milliseconds{200});

        // 连续观察 5 秒。
        // 全程只读取已有反馈，不发送任何控制命令。
        for (int sample = 1; sample <= 50; ++sample)
        {
            const auto positions = gripper.readPositions();

            if (sample == 1 || sample % 5 == 0)
            {
                std::cout << "[sample " << sample << "]\n";

                printPositions(positions);

                std::cout << '\n';
            }

            std::this_thread::sleep_for(std::chrono::milliseconds{100});
        }

        std::cout << "Clean-start feedback test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
