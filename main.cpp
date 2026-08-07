#include "can/can_frame.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/can/can_protocol.hpp"

#include <array>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>

namespace
{
    // 打印 CAN 帧的 ID、数据长度和有效数据。
    void printFrame(const robot::can::CanFrame &frame)
    {
        std::cout << "CAN ID: 0x" << std::hex << frame.id << '\n';
        std::cout << "Length: " << std::dec << static_cast<int>(frame.data_length) << '\n';
        std::cout << "Data: ";
        for (std::size_t i = 0; i < frame.data_length; ++i)
        {
            std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(frame.data[i]) << ' ';
        }
        std::cout << std::dec << '\n';
    }
}

int main()
{
    // 加载并检查所有 TIAGo CAN 总线配置。
    constexpr std::array<const char *, 8> files{"left_elbow.yaml", "left_gripper.yaml", "left_shoulder.yaml", "left_wrist.yaml", "right_elbow.yaml", "right_gripper.yaml", "right_shoulder.yaml", "right_wrist.yaml"};

    try
    {
        for (const auto *file : files)
        {
            const auto path = std::filesystem::path{"config/tiago/can"} / file;

            const auto config = robot::tiago::loadCanBusConfig(path);

            std::cout << "[OK] " << file << " -> " << config.interface_name << ", joints=" << config.joints.size() << '\n';

            for (const auto &joint : config.joints)
            {
                std::cout << "     node=" << joint.node_id << "  " << joint.name << '\n';
            }
        }
    }
    catch (const std::exception &error)
    {
        std::cerr << "[ERROR] " << error.what() << '\n';

        return 1;
    }

    // 构造并打印一个示例 CAN 帧。
    robot::can::CanFrame frame;

    frame.id = 0x101;
    frame.data_length = 8;

    frame.data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    printFrame(frame);

    // 测试协议命令编码和反馈帧解码。
    try
    {
        // 测试 Enable 控制命令。
        const auto enable_frame = robot::tiago::encodeControlCommand(1, robot::tiago::MotorControlCommand::Enable);
        std::cout << "=== Enable Command ===\n";
        printFrame(enable_frame);

        // 测试目标位置为 1000、速度限制为 200 的位置命令。
        const auto position_frame = robot::tiago::encodePositionCommand(1, 1000, 200);
        std::cout << "=== Position Command ===\n";
        printFrame(position_frame);

        // 手工模拟一帧位置为 -1000、速度为 -200 且已使能的电机反馈。
        robot::can::CanFrame feedback_frame;
        feedback_frame.id = 0x181;
        feedback_frame.data_length = 8;
        feedback_frame.data = {0x18, 0xFC, 0xFF, 0xFF, 0x38, 0xFF, 0x01, 0x00};

        const auto feedback = robot::tiago::decodeFeedbackFrame(feedback_frame);
        std::cout << "=== Feedback ===\n";
        std::cout << "Node ID: " << static_cast<int>(feedback.node_id) << '\n';
        std::cout << "Position: " << feedback.position_counts << '\n';
        std::cout << "Velocity: " << feedback.velocity_counts_per_second << '\n';
        std::cout << "Enabled: " << feedback.enabled << '\n';
        std::cout << "Faulted: " << feedback.faulted << '\n';
        std::cout << "Timed out: " << feedback.timed_out << '\n';
        std::cout << "Fault code: " << static_cast<int>(feedback.fault_code) << '\n';
    }
    catch (const std::exception &error)
    {
        std::cerr << "Error: " << error.what() << '\n';

        return 1;
    }
    return 0;
}
