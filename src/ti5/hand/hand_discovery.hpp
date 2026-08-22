#pragma once

#include "ti5/hand/hand_config.hpp"

#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace robot::ti5::hand
{

struct HandInterfaceDiscoveryResult
{
    std::string interface_name;
    bool opened{false};
    std::set<std::uint8_t> confirmed_hand_ids;
    std::string error;
};

struct HandDiscoveryResult
{
    bool success{false};
    std::optional<std::string> left_interface;
    std::optional<std::string> right_interface;
    std::vector<HandInterfaceDiscoveryResult> interfaces;
    std::vector<std::string> errors;
};

class HandDiscovery final
{
public:
    HandDiscoveryResult discover(
        const HandConfig &config,
        const std::vector<std::string> &candidate_interfaces) const;

    HandDiscoveryResult resolve(
        const HandConfig &config,
        const std::vector<HandInterfaceDiscoveryResult> &scan_results) const;
};

} // namespace robot::ti5::hand
