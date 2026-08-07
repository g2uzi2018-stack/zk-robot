#include "can_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace
{
    // 读取必需的标量字段，并统一处理缺失、类型错误和转换错误。
    template <typename T>
    T requireScalar(const YAML::Node &parent, const char *key, const std::string &context)
    {
        const auto node = parent[key];
        if (!node)
        {
            throw std::invalid_argument("Missing '" + std::string(key) + "' in " + context);
        }
        if (!node.IsScalar())
        {
            throw std::invalid_argument("'" + std::string(key) + "' must be a scalar in " + context);
        }
        try
        {
            return node.as<T>();
        }
        catch (const YAML::Exception &error)
        {
            throw std::invalid_argument("Invalid '" + std::string(key) + "' in " + context + ": " + error.what());
        }
    }

    // 读取必需的映射字段，例如 limits 或 encoder 配置段。
    YAML::Node requireMap(const YAML::Node &parent, const char *key, const std::string &context)
    {
        const auto node = parent[key];
        if (!node || !node.IsMap())
        {
            throw std::invalid_argument("Missing or invalid '" + std::string(key) + "' section in " + context);
        }
        return node;
    }

    // 将 YAML 中的单位字符串转换为类型安全的关节单位枚举。
    robot::tiago::JointUnit parseJointUnit(const std::string &value, const std::string &context)
    {
        if (value == "radian")
        {
            return robot::tiago::JointUnit::Radian;
        }
        if (value == "meter")
        {
            return robot::tiago::JointUnit::Meter;
        }
        throw std::invalid_argument("Unsupported joint unit '" + value + "' in " + context);
    }

    // 读取并校验关节的位置范围和最大速度。
    robot::tiago::JointLimits parseLimits(const YAML::Node &joint, const std::string &context)
    {
        const auto limits_node = requireMap(joint, "limits", context);
        robot::tiago::JointLimits limits;
        limits.min_position = requireScalar<double>(limits_node, "min_position", context);
        limits.max_position = requireScalar<double>(limits_node, "max_position", context);
        limits.max_velocity = requireScalar<double>(limits_node, "max_velocity", context);
        // 位置上下限必须是有限值，且最小值小于最大值。
        if (!std::isfinite(limits.min_position) || !std::isfinite(limits.max_position) || limits.min_position >= limits.max_position)
        {
            throw std::invalid_argument("Invalid position limits in " + context);
        }
        // 最大速度必须是正的有限值。
        if (!std::isfinite(limits.max_velocity) || limits.max_velocity <= 0.0)
        {
            throw std::invalid_argument("Invalid max_velocity in " + context);
        }
        return limits;
    }

    // 根据关节单位读取对应的旋转或直线编码器参数。
    robot::tiago::EncoderConfig parseEncoder(const YAML::Node &joint, robot::tiago::JointUnit unit, const std::string &context)
    {
        const auto encoder_node = requireMap(joint, "encoder", context);
        // 方向系数只允许使用 1 或 -1。
        const int direction = requireScalar<int>(encoder_node, "direction", context);
        if (direction != 1 && direction != -1)
        {
            throw std::invalid_argument("Encoder direction must be 1 or -1 in " + context);
        }
        // 零点偏移必须是有限值，单位与关节位置单位一致。
        const double zero_offset = requireScalar<double>(encoder_node, "zero_offset", context);
        if (!std::isfinite(zero_offset))
        {
            throw std::invalid_argument("Invalid zero_offset in " + context);
        }
        if (unit == robot::tiago::JointUnit::Radian)
        {
            // 弧度关节使用电机每转计数和减速比进行换算。
            const auto counts = requireScalar<std::uint32_t>(encoder_node, "counts_per_motor_revolution", context);
            const double gear_ratio = requireScalar<double>(encoder_node, "gear_ratio", context);
            if (counts == 0)
            {
                throw std::invalid_argument("counts_per_motor_revolution must be greater than zero in " + context);
            }
            if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0)
            {
                throw std::invalid_argument("gear_ratio must be greater than zero in " + context);
            }
            return robot::tiago::RotaryEncoderConfig{counts, gear_ratio, direction, zero_offset};
        }
        // 米关节使用每米计数进行位置换算。
        const double counts_per_meter = requireScalar<double>(encoder_node, "counts_per_meter", context);
        if (!std::isfinite(counts_per_meter) || counts_per_meter <= 0.0)
        {
            throw std::invalid_argument("counts_per_meter must be greater than zero in " + context);
        }
        return robot::tiago::LinearEncoderConfig{counts_per_meter, direction, zero_offset};
    }
}

namespace robot::tiago
{
    // 加载并校验一份 CAN 总线 YAML 配置。
    CanBusConfig loadCanBusConfig(const std::filesystem::path &config_path)
    {
        // 读取 YAML 文件，并将底层解析错误转换为统一的参数异常。
        YAML::Node root;
        try
        {
            root = YAML::LoadFile(config_path.string());
        }
        catch (const YAML::Exception &error)
        {
            throw std::invalid_argument("Failed to load CAN config '" + config_path.string() + "': " + error.what());
        }
        // 顶层必须是映射，并且当前只支持格式版本 1。
        if (!root.IsMap())
        {
            throw std::invalid_argument("CAN config root must be a map: " + config_path.string());
        }

        // 读取并校验配置格式版本。
        const int schema_version = requireScalar<int>(root, "schema_version", config_path.string());
        if (schema_version != 1)
        {
            throw std::invalid_argument("Unsupported schema_version in " + config_path.string());
        }

        CanBusConfig config;
        // 读取并校验 SocketCAN 接口名称。
        config.interface_name = requireScalar<std::string>(root, "interface", config_path.string());
        if (config.interface_name.empty())
        {
            throw std::invalid_argument("CAN interface must not be empty");
        }

        // joints 必须是至少包含一个元素的序列。
        const auto joints_node = root["joints"];
        if (!joints_node || !joints_node.IsSequence() || joints_node.size() == 0)
        {
            throw std::invalid_argument("'joints' must be a non-empty sequence in " + config_path.string());
        }

        // 用于检查同一条总线上的关节名称和节点 ID 是否重复。
        std::unordered_set<std::string> joint_names;
        std::unordered_set<std::uint16_t> node_ids;

        // 逐个解析关节，并在加入结果前完成字段和唯一性校验。
        for (std::size_t index = 0; index < joints_node.size(); ++index)
        {
            const auto joint_node = joints_node[index];
            if (!joint_node.IsMap())
            {
                throw std::invalid_argument("Invalid joint entry at index " + std::to_string(index));
            }
            const std::string context = "joint[" + std::to_string(index) + "] in " + config_path.string();
            CanJointConfig joint;
            joint.name = requireScalar<std::string>(joint_node, "name", context);
            if (joint.name.empty())
            {
                throw std::invalid_argument("Joint name must not be empty in " + context);
            }
            const auto node_id = requireScalar<std::uint16_t>(joint_node, "node_id", context);
            if (node_id < 1 || node_id > 127)
            {
                throw std::invalid_argument("node_id must be in range 1..127 in " + context);
            }
            const auto unit_string = requireScalar<std::string>(joint_node, "unit", context);
            joint.node_id = node_id;
            joint.unit = parseJointUnit(unit_string, context);
            joint.limits = parseLimits(joint_node, context);
            joint.encoder = parseEncoder(joint_node, joint.unit, context);
            if (!joint_names.insert(joint.name).second)
            {
                throw std::invalid_argument("Duplicate joint name '" + joint.name + "'");
            }
            if (!node_ids.insert(joint.node_id).second)
            {
                throw std::invalid_argument("Duplicate node_id " + std::to_string(joint.node_id));
            }
            // 只有所有字段均通过校验后，才加入总线配置。
            config.joints.push_back(std::move(joint));
        }
        return config;
    }
}
