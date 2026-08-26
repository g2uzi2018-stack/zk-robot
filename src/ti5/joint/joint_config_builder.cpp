#include "ti5/joint/joint_config_builder.hpp"

#include <map>
#include <set>
#include <stdexcept>
#include <string>

namespace robot::ti5
{

std::vector<JointConfig> makeJointConfigs(
    const Ti5RobotConfig &robot_config,
    const JointSafetyConfig &safety_config,
    const KinematicsModelConfig *model)
{
    std::map<std::string, const PhysicalJointConfig *> physical_by_name;
    for (const auto &physical : robot_config.joints)
    {
        if (physical.name.empty() ||
            !physical_by_name.emplace(physical.name, &physical).second)
        {
            throw std::invalid_argument(
                "TI5 robot config contains an empty or duplicate joint name");
        }
    }

    std::set<std::string> selected_names;
    if (model != nullptr)
    {
        for (const auto &[name, transform] : model->joints)
        {
            static_cast<void>(transform);
            if (physical_by_name.find(name) == physical_by_name.end())
            {
                throw std::invalid_argument(
                    "TI5 kinematics model references unknown joint: " + name);
            }
            selected_names.insert(name);
        }
        if (selected_names.empty())
        {
            throw std::invalid_argument(
                "TI5 kinematics model contains no joints");
        }
    }

    std::vector<JointConfig> result;
    result.reserve(model == nullptr
                       ? robot_config.joints.size()
                       : selected_names.size());
    for (const auto &physical : robot_config.joints)
    {
        if (model != nullptr &&
            selected_names.find(physical.name) == selected_names.end())
        {
            continue;
        }

        const auto safety =
            safety_config.position_limits.find(physical.name);
        if (safety == safety_config.position_limits.end())
        {
            throw std::invalid_argument(
                "TI5 safety config is missing joint: " + physical.name);
        }

        JointCoordinateTransform transform{};
        if (model != nullptr)
        {
            const auto found = model->joints.find(physical.name);
            if (found == model->joints.end())
            {
                throw std::logic_error(
                    "TI5 joint model selection became inconsistent");
            }
            transform = found->second;
        }

        result.push_back(JointConfig{
            physical,
            safety->second,
            transform});
    }

    if (model != nullptr && result.size() != selected_names.size())
    {
        throw std::logic_error(
            "TI5 joint config builder did not produce the complete model");
    }
    return result;
}

} // namespace robot::ti5
