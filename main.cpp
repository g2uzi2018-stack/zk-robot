#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/can/encoder_conversion.hpp"
#include "tiago/motor/can_motor.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <variant>

int main()
{
    try
    {
        // ------------------------------------------------------------
        // 1. 加载左肩 CAN 总线配置。
        // ------------------------------------------------------------
        const auto config = robot::tiago::loadCanBusConfig(
            "config/tiago/can/left_shoulder.yaml");

        if (config.joints.empty())
        {
            throw std::runtime_error("No joint found in CAN configuration");
        }

        // 当前只测试左肩第一个关节。
        const auto &joint_config = config.joints.front();

        std::cout << "CAN interface: " << config.interface_name << '\n';

        std::cout << "Joint: " << joint_config.name << '\n';

        std::cout << "Motor node ID: " << joint_config.motor.node_id << "\n\n";

        // ------------------------------------------------------------
        // 2. 创建 CAN 总线。
        // ------------------------------------------------------------
        robot::tiago::CanBus bus(config.interface_name);

        // ------------------------------------------------------------
        // 3. 创建电机。
        // ------------------------------------------------------------
        robot::tiago::CanMotor motor(joint_config.motor, bus);

        // ------------------------------------------------------------
        // 4. 测试参数。
        // ------------------------------------------------------------
        constexpr double kTargetPosition = 0.8; // rad

        constexpr double kVelocityLimit = 0.1; // rad/s

        constexpr double kPositionTolerance = 0.01; // rad

        // 模拟 Controller 的控制周期。
        constexpr auto kControlPeriod = std::chrono::milliseconds{100};

        // 最多运行 100 个周期，也就是约 10 秒。
        constexpr int kMaximumCycles = 100;

        const auto &encoder = std::get<robot::tiago::RotaryEncoderConfig>(
            joint_config.motor.encoder);

        // ------------------------------------------------------------
        // 5. 使能电机。
        // ------------------------------------------------------------
        std::cout << "Enable motor\n";

        motor.enable();

        std::cout << "Target position: " << kTargetPosition << " rad\n";

        std::cout << "Velocity limit: " << kVelocityLimit << " rad/s\n";

        std::cout << "Control period: " << kControlPeriod.count() << " ms\n\n";

        bool target_reached = false;

        // ------------------------------------------------------------
        // 6. 模拟 Controller 周期运行。
        // ------------------------------------------------------------
        for (int cycle = 1; cycle <= kMaximumCycles; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            // --------------------------------------------------------
            // 6.1 每个控制周期都刷新目标。
            //
            // 这就是以后 Controller 最基本的工作模式：
            //
            // Controller
            //     ↓
            // 周期刷新目标
            //     ↓
            // CanMotor
            // --------------------------------------------------------
            motor.commandPosition(kTargetPosition, kVelocityLimit);

            // --------------------------------------------------------
            // 6.2 查询当前电机状态。
            // --------------------------------------------------------
            const auto feedback = motor.queryStatus();

            if (!feedback)
            {
                std::cout << "[" << cycle << "] No feedback\n";
            }
            else
            {
                const double position = robot::tiago::countsToRadians(
                    feedback->position_counts, encoder);

                std::cout
                    << "["
                    << cycle
                    << "] "
                    << "position = "
                    << position
                    << " rad"
                    << ", counts = "
                    << feedback->position_counts
                    << ", velocity = "
                    << feedback->velocity_counts_per_second
                    << " counts/s"
                    << ", enabled = "
                    << feedback->enabled
                    << ", faulted = "
                    << feedback->faulted
                    << ", timed_out = "
                    << feedback->timed_out
                    << '\n';

                // ----------------------------------------------------
                // 6.3 出现故障则停止测试。
                // ----------------------------------------------------
                if (feedback->faulted)
                {
                    std::cout << "\nMotor fault detected.\n";

                    break;
                }

                // ----------------------------------------------------
                // 6.4 判断是否到达目标位置。
                // ----------------------------------------------------
                if (std::abs(position - kTargetPosition) <= kPositionTolerance)
                {
                    std::cout << "\nTarget reached.\n";

                    target_reached = true;
                    break;
                }
            }

            // --------------------------------------------------------
            // 6.5 保持约 100 ms 控制周期。
            //
            // sleep_until 比固定 sleep_for(100ms) 更合理，
            // 因为前面的 CAN 收发本身也消耗了一点时间。
            // --------------------------------------------------------
            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        if (!target_reached)
        {
            std::cout << "\nTarget was not reached within test time.\n";
        }

        // ------------------------------------------------------------
        // 7. 最后测试一次 readPosition()。
        // ------------------------------------------------------------
        const auto final_position = motor.readPosition();

        if (final_position)
        {
            std::cout << "Final readPosition(): " << *final_position << " rad\n";
        }
        else
        {
            std::cout << "Final readPosition(): no feedback\n";
        }

        // ------------------------------------------------------------
        // 8. 测试结束，禁用电机。
        // ------------------------------------------------------------
        std::cout << "\nDisable motor\n";

        motor.disable();

        std::cout << "Test finished\n";
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }

    return 0;
}
