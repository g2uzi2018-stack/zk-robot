#include "input/exoskeleton/exoskeleton.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

namespace
{

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int)
{
    stop_requested = 1;
}

const char *yesNo(const bool value)
{
    return value ? "yes" : "no";
}

void printJoystick(
    const char *name,
    const robot::input::exoskeleton::JoystickState &joystick)
{
    std::cout << name << " joystick:\n"
              << "  x: " << joystick.x << " (raw " << joystick.raw_x << ")\n"
              << "  y: " << joystick.y << " (raw " << joystick.raw_y << ")\n"
              << "  trigger: " << joystick.trigger
              << " (raw " << joystick.trigger_raw << ")\n"
              << "  buttons raw: 0x"
              << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned int>(joystick.buttons_raw)
              << "\n  extended buttons raw: 0x"
              << std::setw(2)
              << static_cast<unsigned int>(joystick.extended_buttons_raw)
              << std::dec << std::setfill(' ') << "\n";
}

void printImu(
    const char *name,
    const robot::input::exoskeleton::ImuState &imu)
{
    std::cout << name << " IMU:\n"
              << "  acceleration: ["
              << imu.acceleration[0] << ", "
              << imu.acceleration[1] << ", "
              << imu.acceleration[2] << "]\n"
              << "  angular velocity raw: ["
              << imu.angular_velocity_raw[0] << ", "
              << imu.angular_velocity_raw[1] << ", "
              << imu.angular_velocity_raw[2] << "]\n"
              << "  quaternion WXYZ: ["
              << imu.quaternion_wxyz[0] << ", "
              << imu.quaternion_wxyz[1] << ", "
              << imu.quaternion_wxyz[2] << ", "
              << imu.quaternion_wxyz[3] << "]\n"
              << "  present: " << yesNo(imu.present)
              << ", valid: " << yesNo(imu.valid) << "\n";
}

void printState(const robot::input::exoskeleton::ExoskeletonState &state)
{
    std::cout << "Left joints:\n  ";
    for (const double value : state.left_joint_rad)
    {
        std::cout << value << ' ';
    }
    std::cout << "\nRight joints:\n  ";
    for (const double value : state.right_joint_rad)
    {
        std::cout << value << ' ';
    }
    std::cout << "\n"
              << "Mode raw: left=0x" << std::hex
              << static_cast<unsigned int>(state.left_mode_raw)
              << ", right=0x"
              << static_cast<unsigned int>(state.right_mode_raw)
              << std::dec << "\n";

    printJoystick("Left", state.left);
    printJoystick("Right", state.right);
    printImu("Torso", state.torso_imu);
    printImu("Head", state.head_imu);
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " config/exoskeleton.yaml\n";
        return EXIT_FAILURE;
    }

    try
    {
        using namespace std::chrono_literals;
        using robot::input::exoskeleton::Exoskeleton;
        using robot::input::exoskeleton::ExoskeletonStatistics;

        const auto config = robot::input::exoskeleton::loadExoskeletonConfig(
            argv[1]);
        Exoskeleton exoskeleton{config};

        std::signal(SIGINT, requestStop);
        std::signal(SIGTERM, requestStop);
        exoskeleton.start();

        auto previous_time = std::chrono::steady_clock::now();
        ExoskeletonStatistics previous_statistics{};
        while (stop_requested == 0)
        {
            std::this_thread::sleep_for(200ms);

            const auto now = std::chrono::steady_clock::now();
            const auto current_statistics = exoskeleton.statistics();
            const double elapsed = std::chrono::duration<double>(
                now - previous_time).count();
            const double frame_rate = elapsed > 0.0
                                          ? static_cast<double>(
                                                current_statistics.valid_frames -
                                                previous_statistics.valid_frames) /
                                            elapsed
                                          : 0.0;
            previous_time = now;
            previous_statistics = current_statistics;

            std::cout << std::fixed << std::setprecision(3)
                      << "\nConnected: " << yesNo(exoskeleton.connected())
                      << "\nFresh: " << yesNo(exoskeleton.stateFresh())
                      << "\nFrame rate: " << frame_rate << " Hz\n"
                      << "Bytes: " << current_statistics.received_bytes
                      << ", valid frames: " << current_statistics.valid_frames
                      << ", checksum failures: "
                      << current_statistics.checksum_failures
                      << ", tail failures: "
                      << current_statistics.tail_failures << "\n";

            const auto state = exoskeleton.latestState();
            if (state)
            {
                printState(*state);
            }
            else
            {
                std::cout << "No complete valid frame received yet.\n";
            }
            std::cout << std::flush;
        }

        exoskeleton.stop();
        return EXIT_SUCCESS;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Exoskeleton monitor failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
