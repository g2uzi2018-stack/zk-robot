#include "ti5/hand/hand_discovery.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

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
        using namespace robot::ti5::hand;

        HandConfig config;
        config.left.controller_node_id = 70;
        config.left.discovery_enabled = true;
        config.right.controller_node_id = 60;
        config.right.discovery_enabled = true;

        HandInterfaceDiscoveryResult left;
        left.interface_name = "can6";
        left.opened = true;
        left.confirmed_hand_ids.insert(70);

        HandInterfaceDiscoveryResult right;
        right.interface_name = "can7";
        right.opened = true;
        right.confirmed_hand_ids.insert(60);

        HandDiscovery discovery;
        const auto resolved = discovery.resolve(config, {left, right});
        expect(resolved.success, "left/right discovery mapping failed");
        expect(
            resolved.left_interface &&
                *resolved.left_interface == "can6",
            "left hand was bound to the wrong interface");
        expect(
            resolved.right_interface &&
                *resolved.right_interface == "can7",
            "right hand was bound to the wrong interface");

        auto duplicate_left = left;
        duplicate_left.interface_name = "can4";
        const auto ambiguous =
            discovery.resolve(config, {left, duplicate_left, right});
        expect(
            !ambiguous.success &&
                ambiguous.left_interface == std::nullopt,
            "duplicate left-hand response was not rejected");

        const auto missing =
            discovery.resolve(config, {left});
        expect(
            !missing.success &&
                missing.right_interface == std::nullopt,
            "missing right-hand response was not reported");

        std::cout << "Aoyi hand discovery mapping tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Aoyi hand discovery test failed: "
                  << error.what() << '\n';
        return 1;
    }
}
