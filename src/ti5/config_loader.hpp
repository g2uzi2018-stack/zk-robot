#pragma once

#include "ti5/config.hpp"
#include "ti5/config/config_loader.hpp"

#include <filesystem>

namespace robot::ti5
{
    // 将独立配置层中的 CanConfig 适配为 Discovery 使用的运行时参数。
    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path);
}
