#include "tiago/can/can_config.hpp"

#include <iostream>

int main()
{
    try
    {
        const auto config = robot::tiago::loadCanBusConfig("config/tiago/can/left_shoulder.yaml");

        std::cout << "CAN interface: " << config.interface_name << "\n\n";
        std::cout << "Joint count: " << config.joints.size() << "\n";

        for (const auto &joint : config.joints)
        {
            std::cout << "====================\n";
            std::cout << "Joint name: " << joint.name << "\n";
            std::cout << "Limits:\n";
            std::cout << "  min position: " << joint.limits.min_position << "\n";
            std::cout << "  max position: " << joint.limits.max_position << "\n";
            std::cout << "  max velocity: " << joint.limits.max_velocity << "\n\n";
            std::cout << "Motor:\n";
            std::cout << "  node id: " << joint.motor.node_id << "\n";
            std::cout << "  unit: ";

            if (joint.motor.unit == robot::tiago::JointUnit::Radian)
            {
                std::cout << "radian\n";
            }
            else
            {
                std::cout << "meter\n";
            }
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << "\n";
        return 1;
    }

    return 0;
}
