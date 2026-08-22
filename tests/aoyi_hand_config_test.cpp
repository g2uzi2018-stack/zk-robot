#include "ti5/hand/hand_config.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

#ifndef TI5_SOURCE_DIR
#error "TI5_SOURCE_DIR must be defined for this test"
#endif

namespace
{

void expect(const bool condition, const std::string &message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

} // namespace

int main()
{
    try
    {
        const auto config = robot::ti5::hand::loadHandConfig(
            std::string{TI5_SOURCE_DIR} +
            "/config/ti5/t170c/hands.yaml");

        expect(
            config.transport.manage_linux_link,
            "hand transport must manage the Linux CAN link");
        expect(
            config.transport.bitrate == 1000000 &&
                config.transport.restart_ms == 100,
            "hand transport bitrate/restart-ms is incorrect");
        expect(
            config.transport.adapter_selector.kind == "id_path" &&
                config.transport.adapter_selector.expected_channels == 4,
            "hand adapter selector is incorrect");
        expect(
            config.left.controller_node_id == 70 &&
                config.right.controller_node_id == 60,
            "left/right HAND_ID values are incorrect");
        expect(
            config.left.discovery_enabled &&
                config.right.discovery_enabled,
            "both hands must participate in read-only discovery");
        expect(
            !config.left.control_enabled &&
                !config.right.control_enabled,
            "hand control must remain disabled");

        std::cout << "Aoyi hand config tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Aoyi hand config test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
