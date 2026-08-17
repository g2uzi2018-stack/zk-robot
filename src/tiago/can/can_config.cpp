#include "tiago/can/can_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace
{
    // 读取必需的标量字段，并检查字段是否存在且类型正确。
    template <typename T>
    T requireScalar(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!value.IsScalar())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be scalar in " + context);
        }
        return value.as<T>();
    }

    // 读取必需的映射字段，例如 limits、motor 或 encoder 配置段。
    YAML::Node requireMap(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!value.IsMap())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be map in " + context);
        }
        return value;
    }

    // 将 YAML 中的单位字符串转换为类型安全的关节单位枚举。
    robot::tiago::JointUnit parseUnit(const std::string &value)
    {
        if (value == "radian")
        {
            return robot::tiago::JointUnit::Radian;
        }
        if (value == "meter")
        {
            return robot::tiago::JointUnit::Meter;
        }
        throw std::invalid_argument("Unsupported unit: " + value);
    }

    // 读取并校验关节的位置范围和最大速度。
    robot::tiago::JointLimits parseLimits(const YAML::Node &joint_node, const std::string &context)
    {
        const auto limits = requireMap(joint_node, "limits", context);
        robot::tiago::JointLimits result;
        result.min_position = requireScalar<double>(limits, "min_position", context);
        result.max_position = requireScalar<double>(limits, "max_position", context);
        result.max_velocity = requireScalar<double>(limits, "max_velocity", context);

        // 位置下限必须小于位置上限。
        if (result.min_position >= result.max_position)
        {
            throw std::invalid_argument("Invalid position limit in " + context);
        }
        // 最大速度必须为正数。
        if (result.max_velocity <= 0.0)
        {
            throw std::invalid_argument("Invalid max velocity in " + context);
        }
        return result;
    }

    // 根据电机单位读取对应的旋转或直线编码器参数。
    robot::tiago::EncoderConfig parseEncoder(const YAML::Node &motor_node, robot::tiago::JointUnit unit, const std::string &context)
    {
        const auto encoder = requireMap(motor_node, "encoder", context);
        const int direction = requireScalar<int>(encoder, "direction", context);

        // 编码器方向只允许为 1 或 -1。
        if (direction != 1 && direction != -1)
        {
            throw std::invalid_argument("Direction must be 1 or -1");
        }

        // 零点偏移的单位与电机单位一致。
        const double zero_offset = requireScalar<double>(encoder, "zero_offset", context);

        if (unit == robot::tiago::JointUnit::Radian)
        {
            robot::tiago::RotaryEncoderConfig result;
            result.counts_per_motor_revolution = requireScalar<std::uint32_t>(encoder, "counts_per_motor_revolution", context);
            result.gear_ratio = requireScalar<double>(encoder, "gear_ratio", context);
            result.direction = direction;
            result.zero_offset = zero_offset;
            return result;
        }

        // 直线关节使用每米计数进行位置换算。
        robot::tiago::LinearEncoderConfig result;
        result.counts_per_meter = requireScalar<double>(encoder, "counts_per_meter", context);
        result.direction = direction;
        result.zero_offset = zero_offset;
        return result;
    }

    // 读取一个关节对应的 CAN 电机配置。
    robot::tiago::CanMotorConfig parseMotor(const YAML::Node &joint_node, const std::string &context)
    {
        const auto motor = requireMap(joint_node, "motor", context);
        robot::tiago::CanMotorConfig result;
        result.node_id = requireScalar<std::uint16_t>(motor, "node_id", context);
        const auto unit = requireScalar<std::string>(motor, "unit", context);
        result.unit = parseUnit(unit);
        result.encoder = parseEncoder(motor, result.unit, context);
        return result;
    }

    // 读取一个关节的名称、机械限制和电机配置。
    robot::tiago::JointConfig parseJoint(const YAML::Node &node, const std::string &context)
    {
        robot::tiago::JointConfig result;
        result.name = requireScalar<std::string>(node, "name", context);
        result.limits = parseLimits(node, context);
        result.motor = parseMotor(node, context);
        return result;
    }
}

namespace robot::tiago
{
    // 加载并解析一份 CAN 总线 YAML 配置。
    CanBusConfig loadCanBusConfig(const std::filesystem::path &config_path)
    {
        const auto root = YAML::LoadFile(config_path.string());
        CanBusConfig result;
        result.interface_name = requireScalar<std::string>(root, "interface", config_path.string());

        const auto joints = root["joints"];
        // joints 必须存在且为关节配置序列。
        if (!joints || !joints.IsSequence())
        {
            throw std::invalid_argument("joints must be sequence");
        }

        // 用于检查同一条总线上的关节名称是否重复。
        std::unordered_set<std::string> names;

        for (std::size_t i = 0; i < joints.size(); ++i)
        {
            const auto context = "joint[" + std::to_string(i) + "]";
            auto joint = parseJoint(joints[i], context);

            if (!names.insert(joint.name).second)
            {
                throw std::invalid_argument("Duplicate joint name");
            }

            // 只有通过字段解析和名称唯一性检查后，才加入结果配置。
            result.joints.push_back(std::move(joint));
        }
        return result;
    }
}
