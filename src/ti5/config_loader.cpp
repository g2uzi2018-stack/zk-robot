#include "ti5/config_loader.hpp"

#include <stdexcept>

namespace robot::ti5
{
    DiscoveryOptions loadDiscoveryConfig(const std::filesystem::path &config_path)
    {
        const auto config = loadCanConfig(config_path);

        // 这是旧 Discovery 原型的兼容适配，不属于独立配置对象本身。
        DiscoveryOptions result;
        result.interface_regex = config.socketcan.interface_regex;
        result.require_interface_up = config.socketcan.require_interface_up;
        result.response_timeout = config.discovery.response_timeout;
        result.confirmations_required = config.discovery.confirmations_required;
        result.max_attempts = config.discovery.max_attempts;
        result.allow_partial_bus = config.discovery.allow_partial_bus;
        result.require_unique_bus_match = config.discovery.require_unique_bus_match;

        // 保持旧 Discovery 可执行文件原有的运行时策略约束；独立配置层只校验通用格式和值域。
        if (result.interface_regex != "^can[0-9]+$")
        {
            throw std::invalid_argument("socketcan.interface_regex must be '^can[0-9]+$' in " + config_path.string());
        }
        if (!result.require_interface_up)
        {
            throw std::invalid_argument("socketcan.require_interface_up must be true in " + config_path.string());
        }
        if (!config.discovery.enabled)
        {
            throw std::invalid_argument("discovery.enabled must be true for ti5_can_discovery");
        }
        if (config.discovery.allow_partial_bus)
        {
            throw std::invalid_argument("discovery.allow_partial_bus must be false for ti5_can_discovery");
        }
        if (!config.discovery.require_unique_bus_match)
        {
            throw std::invalid_argument("discovery.require_unique_bus_match must be true for ti5_can_discovery");
        }
        return result;
    }
}
