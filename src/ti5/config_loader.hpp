#pragma once

#include "ti5/config/config_loader.hpp"
#include "ti5/discovery.hpp"

#include <filesystem>

namespace robot::ti5
{
    // 兼容现有 Discovery 入口；新的配置对象定义在 ti5/config/config.hpp。
    using Ti5RobotConfig = T170cRobotConfig;

    // 将独立配置层中的 CanConfig 适配为旧 Discovery 原型使用的运行时参数。
    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path);
}
