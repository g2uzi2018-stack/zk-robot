#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/motor/can_motor.hpp"

#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    constexpr double kWheelRadius = 0.0985;
    constexpr double kWheelSeparation = 0.4044;

    // 底盘角速度 rad/s。
    constexpr double kAngularVelocity = 0.5;

    // 原地旋转时单轮角速度。
    constexpr double kWheelVelocity =
        kAngularVelocity *
        kWheelSeparation /
        (2.0 * kWheelRadius);

    // 旋转90度所需时间。
    constexpr double kTurnDurationSeconds =
        (kPi / 2.0) /
        kAngularVelocity;

    constexpr auto kControlPeriod =
        std::chrono::milliseconds{100};

    robot::tiago::CanMotorConfig makeMotorConfig(
        std::uint16_t node_id)
    {
        robot::tiago::RotaryEncoderConfig encoder;

        encoder.counts_per_motor_revolution = 4096;
        encoder.gear_ratio = 1.0;
        encoder.direction = 1;
        encoder.zero_offset = 0.0;

        robot::tiago::CanMotorConfig config;

        config.node_id = node_id;
        config.unit =
            robot::tiago::JointUnit::Radian;

        config.encoder = encoder;

        return config;
    }

    void driveFor(
        robot::tiago::CanMotor &right_motor,
        robot::tiago::CanMotor &left_motor,
        double right_velocity,
        double left_velocity,
        double duration_seconds)
    {
        const auto start =
            std::chrono::steady_clock::now();

        const auto duration =
            std::chrono::duration<double>{
                duration_seconds};

        int cycle = 0;

        while (std::chrono::steady_clock::now() -
                   start <
               duration)
        {
            const auto cycle_start =
                std::chrono::steady_clock::now();

            right_motor.commandVelocity(
                right_velocity);

            left_motor.commandVelocity(
                left_velocity);

            ++cycle;

            if (cycle % 5 == 0)
            {
                const auto right =
                    right_motor.readVelocity();

                const auto left =
                    left_motor.readVelocity();

                std::cout
                    << "  right: ";

                if (right)
                    std::cout << *right;
                else
                    std::cout << "no feedback";

                std::cout
                    << " rad/s, left: ";

                if (left)
                    std::cout << *left;
                else
                    std::cout << "no feedback";

                std::cout << " rad/s\n";
            }

            std::this_thread::sleep_until(
                cycle_start +
                kControlPeriod);
        }
    }

    void stopAndWait(
        robot::tiago::CanMotor &right_motor,
        robot::tiago::CanMotor &left_motor)
    {
        right_motor.stop();
        left_motor.stop();

        std::this_thread::sleep_for(
            std::chrono::seconds{1});
    }
}

int main()
{
    try
    {
        robot::tiago::CanBus bus{
            "vcan8"};

        robot::tiago::CanMotor right_motor{
            makeMotorConfig(10),
            bus};

        robot::tiago::CanMotor left_motor{
            makeMotorConfig(11),
            bus};

        std::cout
            << "Base rotation test\n\n";

        std::cout
            << "Wheel velocity: "
            << kWheelVelocity
            << " rad/s\n";

        std::cout
            << "90 degree duration: "
            << kTurnDurationSeconds
            << " s\n";

        std::cout
            << "\nClear faults\n";

        right_motor.clearFault();
        left_motor.clearFault();

        std::cout
            << "Enable wheels\n";

        right_motor.enable();
        left_motor.enable();

        // ============================================================
        // 左转90度
        // ============================================================
        std::cout
            << "\n====================================\n"
            << "LEFT TURN 90 DEGREES\n"
            << "====================================\n";

        driveFor(
            right_motor,
            left_motor,
            +kWheelVelocity,
            -kWheelVelocity,
            kTurnDurationSeconds);

        stopAndWait(
            right_motor,
            left_motor);

        std::cout
            << "\nLeft turn finished.\n"
            << "Robot should now face about 90 degrees left.\n";

        // ============================================================
        // 右转90度，回到原方向
        // ============================================================
        std::cout
            << "\n====================================\n"
            << "RETURN TO ORIGINAL HEADING\n"
            << "====================================\n";

        driveFor(
            right_motor,
            left_motor,
            -kWheelVelocity,
            +kWheelVelocity,
            kTurnDurationSeconds);

        stopAndWait(
            right_motor,
            left_motor);

        std::cout
            << "\nRobot should now face approximately "
            << "the original direction.\n";

        // ============================================================
        // 收尾
        // ============================================================
        right_motor.disable();
        left_motor.disable();

        std::cout
            << "\n====================================\n"
            << "BASE ROTATION TEST FINISHED\n"
            << "====================================\n";

        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Error: "
            << error.what()
            << '\n';

        return 1;
    }
}