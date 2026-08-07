#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <variant>
#include <vector>

namespace robot::tiago
{
    // 关节位置、速度和编码器零点偏移使用的物理单位。
    enum class JointUnit
    {
        // 旋转关节单位：弧度。
        Radian,
        // 直线关节单位：米。
        Meter
    };

    // 单个关节的运动范围和速度限制。
    struct JointLimits
    {
        // 允许的最小位置，单位由 CanJointConfig::unit 决定。
        double min_position{0.0};
        // 允许的最大位置，单位由 CanJointConfig::unit 决定。
        double max_position{0.0};
        // 允许的最大速度，单位由 CanJointConfig::unit 决定，并按秒计。
        double max_velocity{0.0};
    };

    // 旋转关节的编码器换算参数。
    struct RotaryEncoderConfig
    {
        // 电机每转对应的编码器计数。
        std::uint32_t counts_per_motor_revolution{0};
        // 电机与关节之间的减速比。
        double gear_ratio{0.0};
        // 编码器方向系数，1 表示正向，-1 表示反向。
        int direction{1};
        // 编码器零点偏移，单位：弧度。
        double zero_offset{0.0};
    };

    // 直线关节的编码器换算参数。
    struct LinearEncoderConfig
    {
        // 每米对应的编码器计数。
        double counts_per_meter{0.0};
        // 编码器方向系数，1 表示正向，-1 表示反向。
        int direction{1};
        // 编码器零点偏移，单位：米。
        double zero_offset{0.0};
    };

    // 表示 EncoderConfig 在任意时刻可以保存：一个 RotaryEncoderConfig或一个 LinearEncoderConfig
    using EncoderConfig = std::variant<RotaryEncoderConfig, LinearEncoderConfig>;

    // 单个 CAN 关节的完整配置。
    struct CanJointConfig
    {
        // 关节名称，通常需要与机器人模型中的名称保持一致。
        std::string name;
        // CAN 节点 ID。
        std::uint16_t node_id{0};
        // 关节位置和速度使用的单位。
        JointUnit unit{JointUnit::Radian};

        // 关节的位置和速度限制。
        JointLimits limits;
        // 与关节单位匹配的编码器换算配置。
        EncoderConfig encoder;
    };

    // 单条 CAN 总线的配置，包括总线接口和该总线上的关节。
    struct CanBusConfig
    {
        // SocketCAN 接口名称，例如 vcan0。
        std::string interface_name;
        // 该 CAN 总线上的关节配置列表。
        std::vector<CanJointConfig> joints;
    };

    // 从 YAML 配置文件加载一条 CAN 总线的配置。
    CanBusConfig loadCanBusConfig(
        const std::filesystem::path &config_path);
}
