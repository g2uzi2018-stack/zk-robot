#include "ti5/config/config_loader.hpp"
#include "ti5/hand/hand_config.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

std::filesystem::path defaultConfigRoot()
{
#ifdef TI5_SOURCE_DIR
    return std::filesystem::path{TI5_SOURCE_DIR} /
           "config" / "ti5" / "t170c";
#else
    return std::filesystem::path{"config"} / "ti5" / "t170c";
#endif
}

void printUsage(const char *program)
{
    std::cout
        << "用法：\n"
        << "  " << program << " [--config-root 配置目录]\n\n"
        << "默认只加载并检查 TI5 配置，不打开 CAN，也不会让机器人运动。\n";
}

std::filesystem::path parseConfigRoot(const int argc, char **argv)
{
    if (argc == 1)
    {
        return defaultConfigRoot();
    }
    if (argc == 2 &&
        (std::string{argv[1]} == "--help" ||
         std::string{argv[1]} == "-h"))
    {
        printUsage(argv[0]);
        std::exit(0);
    }
    if (argc == 3 && std::string{argv[1]} == "--config-root")
    {
        return std::filesystem::path{argv[2]};
    }
    throw std::invalid_argument("参数错误，请使用 --help 查看用法");
}

} // namespace

int main(const int argc, char **argv)
{
    try
    {
        const auto config_root = parseConfigRoot(argc, argv);
        const auto robot = robot::ti5::loadRobotConfig(
            config_root / "robot.yaml");
        const auto can = robot::ti5::loadCanConfig(
            config_root / "can.yaml");
        const auto safety = robot::ti5::loadJointSafetyConfig(
            config_root / "safety.yaml");
        const auto kinematics = robot::ti5::loadKinematicsConfig(
            config_root / "kinematics.yaml");
        const auto hands = robot::ti5::hand::loadHandConfig(
            config_root / "hands.yaml");

        std::cout
            << "zk_robot main smoke test 通过\n"
            << "  robot: " << robot.vendor << " " << robot.model << "\n"
            << "  body motors: " << robot.body_motor_count << "\n"
            << "  CAN buses: " << robot.can_buses.size() << "\n"
            << "  safety limits: " << safety.position_limits.size() << "\n"
            << "  kinematics models: " << kinematics.models.size() << "\n"
            << "  CAN bitrate: " << can.socketcan.bitrate << "\n"
            << "  hands: " << hands.left.name << ", "
            << hands.right.name << "\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "zk_robot main smoke test 失败："
                  << error.what() << '\n';
        return 1;
    }
}
