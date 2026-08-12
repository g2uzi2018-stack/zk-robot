#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

namespace robot::tiago
{
    // 关节位置和速度使用的物理单位。
    enum class JointUnit
    {
        Radian,
        Meter
    };

    // 关节的机械位置范围和最大速度。
    struct JointLimits
    {
        double min_position{0.0};
        double max_position{0.0};
        double max_velocity{0.0};
    };

    // 旋转关节的编码器换算参数。
    struct RotaryEncoderConfig
    {
        // 电机每转对应的编码器计数。
        std::uint32_t counts_per_motor_revolution{0};
        // 电机与关节之间的减速比。
        double gear_ratio{0.0};
        // 编码器方向，1 表示正向，-1 表示反向。
        int direction{1};
        // 编码器零点偏移，单位为弧度。
        double zero_offset{0.0};
    };

    // 直线关节的编码器换算参数。
    struct LinearEncoderConfig
    {
        // 每米对应的编码器计数。
        double counts_per_meter{0.0};
        // 编码器方向，1 表示正向，-1 表示反向。
        int direction{1};
        // 编码器零点偏移，单位为米。
        double zero_offset{0.0};
    };

    // 根据关节类型保存旋转或直线编码器配置。
    using EncoderConfig = std::variant<RotaryEncoderConfig, LinearEncoderConfig>;

    // 一个 CAN 节点对应的电机配置。
    //
    // 该配置不包含关节自身的机械位置和速度限制。
    struct CanMotorConfig
    {
        // CAN 节点 ID。
        std::uint16_t node_id{0};
        // 电机位置和速度使用的单位。
        JointUnit unit{JointUnit::Radian};
        // 与关节单位匹配的编码器参数。
        EncoderConfig encoder;
    };

    // 一个机器人关节的完整配置。
    //
    // 包含关节名称、机械限制以及对应的 CAN 电机配置。
    struct JointConfig
    {
        // 关节名称，通常与机器人模型中的名称保持一致。
        std::string name;
        // 关节的机械位置和速度限制。
        JointLimits limits;
        // 与该关节关联的 CAN 电机配置。
        CanMotorConfig motor;
    };

    // 一条 CAN 总线的配置，包括接口名称和该总线上的关节列表。
    struct CanBusConfig
    {
        // SocketCAN 接口名称，例如 vcan0。
        std::string interface_name;
        // 该 CAN 总线上的关节配置列表。
        std::vector<JointConfig> joints;
    };

    // 从 YAML 配置文件加载并返回一条 CAN 总线的配置。
    CanBusConfig loadCanBusConfig(const std::filesystem::path &config_path);
}
