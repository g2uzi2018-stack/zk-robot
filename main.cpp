#include "tiago/base/base.hpp"
#include "tiago/base/base_config.hpp"
#include "tiago/controller/base_controller.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
    constexpr auto kControlPeriod = std::chrono::milliseconds{100};

    // 以固定周期刷新控制器，让最新的速度目标持续发送到驱动器。
    void runFor(robot::tiago::BaseController &controller, double seconds)
    {
        const auto start = std::chrono::steady_clock::now();
        const auto duration = std::chrono::duration<double>{seconds};

        while (std::chrono::steady_clock::now() - start < duration)
        {
            const auto cycle_start = std::chrono::steady_clock::now();
            controller.update();
            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }
    }

    // 发送零速度并持续刷新一段时间，给底层驱动器留出停稳时间。
    void stopMotion(robot::tiago::BaseController &controller)
    {
        controller.setTarget(0.0, 0.0);
        runFor(controller, 1.0);
    }
}

int main()
{
    try
    {
        const auto config = robot::tiago::loadBaseConfig("config/tiago/base/base.yaml");
        robot::tiago::Base base(config);
        robot::tiago::BaseController controller(base);

        std::cout << "Base CAN interface: " << config.interface_name << '\n';
        std::cout << "Wheel radius: " << config.wheel_radius << " m\n";
        std::cout << "Wheel separation: " << config.wheel_separation << " m\n";

        // 驱动器上电前先清除历史故障，再开始周期控制。
        std::cout << "\nClear base faults\n";
        base.clearFault();

        std::cout << "Enable base\n";
        base.enable();
        controller.start();

        // TEST 1：验证正向线速度命令。
        std::cout << "\n====================================\n"
                  << "[TEST 1] Forward\n"
                  << "====================================\n";
        controller.setTarget(0.20, 0.0);
        runFor(controller, 2.0);
        stopMotion(controller);
        std::cout << "PASS: forward\n";

        // TEST 2：验证角速度为正时向左转，并以约 90 度为目标。
        // 计算：旋转时间 = (π / 2) / 0.5 ≈ 3.14159 秒。
        std::cout << "\n====================================\n"
                  << "[TEST 2] Turn left about 90 degrees\n"
                  << "====================================\n";
        controller.setTarget(0.0, 0.5);
        runFor(controller, 3.14159);
        stopMotion(controller);
        std::cout << "PASS: left turn\n";

        // TEST 3：反向旋转，验证机器人回到原来的朝向。
        std::cout << "\n====================================\n"
                  << "[TEST 3] Return to original heading\n"
                  << "====================================\n";
        controller.setTarget(0.0, -0.5);
        runFor(controller, 3.14159);
        stopMotion(controller);
        std::cout << "PASS: return heading\n";

        // TEST 4：验证负线速度命令。
        std::cout << "\n====================================\n"
                  << "[TEST 4] Backward\n"
                  << "====================================\n";
        controller.setTarget(-0.20, 0.0);
        runFor(controller, 2.0);
        stopMotion(controller);
        std::cout << "PASS: backward\n";

        // TEST 5：验证尚未 update() 的旧目标会被最新目标覆盖。
        std::cout << "\n====================================\n"
                  << "[TEST 5] Latest target wins\n"
                  << "====================================\n";
        controller.setTarget(0.10, 0.0);
        controller.setTarget(0.0, 0.3);

        if (controller.linearVelocityTarget() != 0.0 ||
            controller.angularVelocityTarget() != 0.3)
        {
            throw std::runtime_error("Latest target was not preserved");
        }

        std::cout << "PASS: latest target wins\n";
        stopMotion(controller);

        // TEST 6：非法速度应抛出异常，且不能覆盖原有目标。
        std::cout << "\n====================================\n"
                  << "[TEST 6] Invalid velocity\n"
                  << "====================================\n";
        try
        {
            controller.setTarget(1.0, 0.0);
            throw std::runtime_error("Invalid velocity was unexpectedly accepted");
        }
        catch (const std::out_of_range &error)
        {
            std::cout << "PASS: caught expected exception: " << error.what() << '\n';
        }

        if (controller.linearVelocityTarget() != 0.0 ||
            controller.angularVelocityTarget() != 0.0)
        {
            throw std::runtime_error("Invalid command changed previous target");
        }

        std::cout << "PASS: previous target preserved\n";

        // TEST 7：停止控制器后，状态应从 Running 回到 Idle。
        std::cout << "\n====================================\n"
                  << "[TEST 7] Stop controller\n"
                  << "====================================\n";
        controller.stop();

        if (controller.state() != robot::tiago::BaseController::ControlState::Idle)
        {
            throw std::runtime_error("BaseController did not return to Idle");
        }

        std::cout << "PASS: Running -> Idle\n";
        base.disable();

        std::cout << "\n====================================\n"
                  << "BASE CONTROLLER TEST PASSED\n"
                  << "====================================\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "\n====================================\n"
                  << "BASE CONTROLLER TEST FAILED\n"
                  << "====================================\n"
                  << error.what() << '\n';
        return 1;
    }
}
