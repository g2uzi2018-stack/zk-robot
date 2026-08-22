#pragma once

#include "can/can_interface_manager.hpp"

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>

namespace robot::ti5::hand
{

struct HandDiscoveryConfig
{
    std::chrono::milliseconds response_timeout{50};
    std::size_t confirmations_required{2};
    std::size_t max_attempts{3};
};

struct HandTransportConfig
{
    std::string type;
    std::uint32_t bitrate{0};
    std::uint32_t restart_ms{0};
    bool manage_linux_link{false};
    bool require_interface_up{true};
    bool validate_bitrate{true};
    std::chrono::milliseconds reconfigure_wait{100};
    std::chrono::milliseconds startup_wait{100};
    robot::can::CanAdapterSelector adapter_selector;
};

struct HandSideConfig
{
    std::string name;
    std::string protocol;
    std::uint8_t controller_node_id{0};
    bool required_for_body_startup{false};
    bool protocol_verified{false};
    bool discovery_enabled{false};
    bool control_enabled{false};
};

struct HandConfig
{
    HandTransportConfig transport;
    HandDiscoveryConfig discovery;
    HandSideConfig left;
    HandSideConfig right;
};

HandConfig loadHandConfig(const std::filesystem::path &config_path);

} // namespace robot::ti5::hand
