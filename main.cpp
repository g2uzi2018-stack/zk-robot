#include "tiago/can/can_config.hpp"
#include "tiago/controller/torso_controller.hpp"
#include "tiago/torso/torso.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace
{
    constexpr auto kControlPeriod = std::chrono::milliseconds{100};

    constexpr double kPositionTolerance = 0.003; // 3 mm

    const char *stateName(robot::tiago::TorsoController::ControlState state)
    {
        using State = robot::tiago::TorsoController::ControlState;

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

    void printPosition(const std::optional<double> &position)
    {
        std::cout << "  torso_lift_joint: ";

        if (position)
        {
            std::cout << *position << " m\n";
        }
        else
        {
            std::cout << "no feedback\n";
        }
    }

    void runUntilReached(robot::tiago::TorsoController &controller, int max_cycles)
    {
        for (int cycle = 1; cycle <= max_cycles; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            controller.update();

            if (cycle == 1 || cycle % 5 == 0)
            {
                const bool reached = controller.targetReached(kPositionTolerance);

                std::cout << "\n[cycle " << cycle << "]\n";

                std::cout << "Target: " << controller.targetPosition() << " m\n";

                std::cout << "Current position:\n";

                printPosition(controller.currentPosition());

                std::cout << "Target reached: " << (reached ? "yes" : "no") << '\n';

                if (reached)
                {
                    return;
                }
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        throw std::runtime_error("Torso target was not reached");
    }
}

int main()
{
    try
    {
        // ============================================================
        // 1. Create Torso + Controller
        // ============================================================
        const auto config = robot::tiago::loadCanBusConfig("config/tiago/can/torso.yaml");

        robot::tiago::Torso torso(config);
        robot::tiago::TorsoController controller(torso);

        std::cout << "Torso CAN interface: " << config.interface_name << '\n';

        std::cout << "Joint: " << torso.joint().name() << "\n\n";

        // ============================================================
        // 2. Idle feedback
        // ============================================================
        std::this_thread::sleep_for(std::chrono::milliseconds{200});

        controller.update();

        std::cout << "Initial controller state: " << stateName(controller.state()) << '\n';

        std::cout << "Initial position:\n";

        printPosition(controller.currentPosition());

        // ============================================================
        // 3. Hardware lifecycle
        // ============================================================
        std::cout << "\nClear torso faults\n";

        torso.clearFault();

        std::cout << "Enable torso\n";

        torso.enable();

        constexpr double kVelocityLimit = 0.03; // m/s

        // ============================================================
        // 4. TEST 1 - start()
        // ============================================================
        constexpr double kTargetA = 0.22;

        std::cout << "\n====================================\n"
                  << "[TEST 1] TorsoController start -> 0.22 m\n"
                  << "====================================\n";

        controller.start(kTargetA, kVelocityLimit);

        std::cout << "Controller state: " << stateName(controller.state()) << '\n';

        runUntilReached(controller, 150);

        std::cout << "\nPASS: target A reached\n";

        // ============================================================
        // 5. TEST 2 - latest target wins
        //
        // 两次setTarget之间故意不调用update()。
        // 因此0.18不应该真正发送到硬件。
        // ============================================================
        std::cout << "\n====================================\n"
                  << "[TEST 2] Latest target wins\n"
                  << "====================================\n";

        controller.setTarget(0.18, kVelocityLimit);

        controller.setTarget(0.10, kVelocityLimit);

        std::cout << "Latest target: " << controller.targetPosition() << " m\n";

        if (controller.targetPosition() != 0.10)
        {
            throw std::runtime_error("Latest target was not preserved");
        }

        std::cout << "PASS: latest target is 0.10 m\n";

        runUntilReached(controller, 150);

        std::cout << "\nPASS: target B reached\n";

        // ============================================================
        // 6. TEST 3 - invalid position
        // ============================================================
        std::cout << "\n====================================\n"
                  << "[TEST 3] Invalid position target\n"
                  << "====================================\n";

        try
        {
            controller.setTarget(0.50, kVelocityLimit);

            throw std::runtime_error("Invalid position was unexpectedly accepted");
        }
        catch (const std::out_of_range &error)
        {
            std::cout << "PASS: caught expected exception: " << error.what() << '\n';
        }

        if (controller.targetPosition() != 0.10)
        {
            throw std::runtime_error("Invalid position changed previous target");
        }

        std::cout << "PASS: previous target preserved\n";

        // ============================================================
        // 7. TEST 4 - invalid velocity
        // ============================================================
        std::cout << "\n====================================\n"
                  << "[TEST 4] Invalid velocity limit\n"
                  << "====================================\n";

        try
        {
            controller.setTarget(0.20, 0.10); // > YAML max_velocity 0.05

            throw std::runtime_error("Invalid velocity was unexpectedly accepted");
        }
        catch (const std::out_of_range &error)
        {
            std::cout << "PASS: caught expected exception: " << error.what() << '\n';
        }

        if (controller.targetPosition() != 0.10 || controller.velocityLimit() != kVelocityLimit)
        {
            throw std::runtime_error("Invalid velocity changed previous command");
        }

        std::cout << "PASS: previous command preserved\n";

        // ============================================================
        // 8. Hold 3 seconds
        // ============================================================
        std::cout << "\n====================================\n"
                  << "[TEST 5] Hold torso for 3 seconds\n"
                  << "====================================\n";

        for (int cycle = 1; cycle <= 30; ++cycle)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            controller.update();

            if (cycle % 10 == 0)
            {
                std::cout << "\n[hold cycle " << cycle << "]\n";

                printPosition(controller.currentPosition());

                const bool reached = controller.targetReached(kPositionTolerance);

                std::cout << "Target reached: " << (reached ? "yes" : "no") << '\n';

                if (!reached)
                {
                    throw std::runtime_error("Torso lost target during hold");
                }
            }

            std::this_thread::sleep_until(cycle_start + kControlPeriod);
        }

        std::cout << "\nPASS: hold test\n";

        // ============================================================
        // 9. Stop -> Idle
        // ============================================================
        std::cout << "\n====================================\n"
                  << "[TEST 6] Stop controller\n"
                  << "====================================\n";

        controller.stop();

        std::cout << "Controller state after stop: " << stateName(controller.state()) << '\n';

        if (controller.state() != robot::tiago::TorsoController::ControlState::Idle)
        {
            throw std::runtime_error("TorsoController did not return to Idle");
        }

        std::cout << "PASS: Running -> Idle\n";

        // ============================================================
        // 10. Disable hardware
        // ============================================================
        std::cout << "\nDisable torso\n";

        torso.disable();

        std::cout << "\n====================================\n"
                  << "TORSO CONTROLLER TEST PASSED\n"
                  << "====================================\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "\n====================================\n"
                  << "TORSO CONTROLLER TEST FAILED\n"
                  << "====================================\n"
                  << error.what() << '\n';

        return 1;
    }
}
