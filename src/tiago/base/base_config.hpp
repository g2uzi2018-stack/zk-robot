#pragma once

#include "tiago/can/can_config.hpp"

#include <filesystem>
#include <string>

namespace robot::tiago
{
    // 底盘配置，包括 CAN 接口、几何参数、速度限制和左右轮电机参数。
    struct BaseConfig
    {
        // SocketCAN 接口名称，例如 can0 或 vcan0。
        std::string interface_name;

        // 车轮半径，单位为 m。
        double wheel_radius{0.0};
        // 左右车轮接触点之间的距离，单位为 m。
        double wheel_separation{0.0};
        // 底盘线速度绝对值上限，单位为 m/s。
        double max_linear_velocity{0.0};
        // 底盘角速度绝对值上限，单位为 rad/s。
        double max_angular_velocity{0.0};

        // 右轮和左轮的 CAN 电机配置。
        CanMotorConfig right_motor;
        CanMotorConfig left_motor;
    };

    // 从 YAML 文件加载并校验底盘配置。
    BaseConfig loadBaseConfig(const std::filesystem::path &config_path);
}
