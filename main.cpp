#include "can/socket_can.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/motor/can_motor.hpp"

#include <iostream>
#include <stdexcept>


int main()
{
    try
    {
        // 1. 加载左肩 CAN 总线配置。
        const auto config =
            robot::tiago::loadCanBusConfig(
                "config/tiago/can/left_shoulder.yaml");


        if (config.joints.empty())
        {
            throw std::runtime_error(
                "No joint found in CAN configuration");
        }


        // 2. 当前先测试左肩的第一个电机。
        const auto &joint_config =
            config.joints.front();


        std::cout
            << "CAN interface: "
            << config.interface_name
            << '\n';

        std::cout
            << "Joint: "
            << joint_config.name
            << '\n';

        std::cout
            << "Motor node ID: "
            << joint_config.motor.node_id
            << '\n';


        // 3. 打开 YAML 指定的 SocketCAN 总线。
        robot::can::SocketCan can(
            config.interface_name);


        // 4. 使用 JointConfig 中的 motor 配置创建 CanMotor。
        robot::tiago::CanMotor motor(
            joint_config.motor,
            can);


        // 5. 发送使能命令。
        std::cout << "Send enable command\n";
        motor.enable();


        // 6. 发送一个简单的位置命令。
        //
        // 左肩是旋转关节：
        // position       = 0.1 rad
        // velocity_limit = 0.1 rad/s
        std::cout << "Send position command\n";
        motor.commandPosition(
            0.1,
            0.1);


        std::cout << "Done\n";
    }
    catch (const std::exception &error)
    {
        std::cerr
            << "Error: "
            << error.what()
            << '\n';

        return 1;
    }


    return 0;
}
