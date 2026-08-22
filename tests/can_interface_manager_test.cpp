#include "can/can_interface_manager.hpp"

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
    void expect(const bool condition, const std::string &message)
    {
        if (!condition)
        {
            throw std::runtime_error(message);
        }
    }
}

int main()
{
    try
    {
        using robot::can::CanAdapterSelector;
        using robot::can::CanInterfaceInfo;
        using robot::can::CanInterfaceManager;

        CanInterfaceInfo body0;
        body0.name = "can0";
        body0.ifindex = 4;
        body0.id_path = "platform-3610000.usb-usb-0:1.2.2:1.0";
        body0.sysfs_parent = "1-1.2.2:1.0";

        CanInterfaceInfo body1 = body0;
        body1.name = "can1";
        body1.ifindex = 5;

        CanInterfaceInfo hand = body0;
        hand.name = "can4";
        hand.ifindex = 8;
        hand.id_path = "platform-3610000.usb-usb-0:1.2.3:1.0";
        hand.sysfs_parent = "1-1.2.3:1.0";

        const CanInterfaceManager manager;
        const std::vector<CanInterfaceInfo> interfaces{body1, hand, body0};

        const auto selected = manager.selectAdapter(
            interfaces,
            CanAdapterSelector{
                "id_path",
                "platform-3610000.usb-usb-0:1.2.2:1.0",
                2});
        expect(selected.size() == 2, "id_path selector must select both body channels");
        expect(selected[0].name == "can0" && selected[1].name == "can1",
               "selected interfaces must be sorted by ifindex");

        const auto parent_selected = manager.selectAdapter(
            interfaces,
            CanAdapterSelector{"sysfs_parent", "1-1.2.3:1.0", 1});
        expect(parent_selected.size() == 1 && parent_selected.front().name == "can4",
               "sysfs_parent selector must select the hand adapter only");

        bool duplicate_serial_rejected = false;
        try
        {
            static_cast<void>(manager.selectAdapter(
                interfaces,
                CanAdapterSelector{"usb_serial_short", "KCANX4WCID001", 2}));
        }
        catch (const std::exception &)
        {
            duplicate_serial_rejected = true;
        }
        expect(duplicate_serial_rejected,
               "a selector must reject an unexpected channel count caused by duplicate serials");

        std::cout << "CAN interface manager selector tests passed\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "CAN interface manager selector test failed: " << error.what() << '\n';
        return 1;
    }
}
