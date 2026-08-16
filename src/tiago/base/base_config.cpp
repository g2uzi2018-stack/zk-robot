#include "tiago/base/base_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <stdexcept>
#include <string>

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

    // 读取必需的映射字段，例如 geometry、limits 或 motors 配置段。
    YAML::Node requireMap(const YAML::Node &node, const char *key, const std::string &context)
    {
        const auto value = node[key];
        if (!value || !value.IsMap())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be map in " + context);
        }
        return value;
    }

    // 底盘轮电机必须使用旋转编码器和弧度单位。
    robot::tiago::CanMotorConfig parseMotor(const YAML::Node &node, const std::string &context)
    {
        robot::tiago::CanMotorConfig result;
        result.node_id = requireScalar<std::uint16_t>(node, "node_id", context);

        const auto unit = requireScalar<std::string>(node, "unit", context);
        if (unit != "radian")
        {
            throw std::invalid_argument("Base motors must use radian units");
        }
        result.unit = robot::tiago::JointUnit::Radian;

        const auto encoder = requireMap(node, "encoder", context);
        robot::tiago::RotaryEncoderConfig encoder_config;
        encoder_config.counts_per_motor_revolution =
            requireScalar<std::uint32_t>(encoder, "counts_per_motor_revolution", context);
        encoder_config.gear_ratio = requireScalar<double>(encoder, "gear_ratio", context);
        encoder_config.direction = requireScalar<int>(encoder, "direction", context);
        encoder_config.zero_offset = requireScalar<double>(encoder, "zero_offset", context);

        if (encoder_config.counts_per_motor_revolution == 0)
        {
            throw std::invalid_argument("Base encoder resolution must be positive");
        }
        if (encoder_config.gear_ratio <= 0.0)
        {
            throw std::invalid_argument("Base motor gear_ratio must be positive");
        }
        if (encoder_config.direction != 1 && encoder_config.direction != -1)
        {
            throw std::invalid_argument("Base motor direction must be 1 or -1");
        }

        result.encoder = encoder_config;
        return result;
    }
}

namespace robot::tiago
{
    // 读取底盘几何参数、速度限制和左右轮的 CAN 电机配置。
    BaseConfig loadBaseConfig(const std::filesystem::path &config_path)
    {
        const auto root = YAML::LoadFile(config_path.string());
        BaseConfig result;
        result.interface_name = requireScalar<std::string>(root, "interface", config_path.string());

        const auto geometry = requireMap(root, "geometry", config_path.string());
        result.wheel_radius = requireScalar<double>(geometry, "wheel_radius", "geometry");
        result.wheel_separation = requireScalar<double>(geometry, "wheel_separation", "geometry");
        if (result.wheel_radius <= 0.0 || result.wheel_separation <= 0.0)
        {
            throw std::invalid_argument("Base geometry must be positive");
        }

        const auto limits = requireMap(root, "limits", config_path.string());
        result.max_linear_velocity = requireScalar<double>(limits, "max_linear_velocity", "limits");
        result.max_angular_velocity = requireScalar<double>(limits, "max_angular_velocity", "limits");
        if (result.max_linear_velocity <= 0.0 || result.max_angular_velocity <= 0.0)
        {
            throw std::invalid_argument("Base velocity limits must be positive");
        }

        const auto motors = requireMap(root, "motors", config_path.string());
        result.right_motor = parseMotor(requireMap(motors, "right", "motors"), "right motor");
        result.left_motor = parseMotor(requireMap(motors, "left", "motors"), "left motor");

        // 左右轮必须使用不同的节点 ID，避免同一条 CAN 总线上的地址冲突。
        if (result.right_motor.node_id == result.left_motor.node_id)
        {
            throw std::invalid_argument("Base motor node IDs must be different");
        }

        return result;
    }
}
