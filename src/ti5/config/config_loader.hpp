#pragma once

#include "ti5/config/config.hpp"

#include <filesystem>

namespace robot::ti5
{
    // 从 robot.yaml 加载并严格校验 T170C 本体配置。
    T170cRobotConfig loadRobotConfig(const std::filesystem::path &config_path);

    // 从 can.yaml 加载并严格校验当前阶段所需的 CAN 配置。
    CanConfig loadCanConfig(const std::filesystem::path &config_path);
}
