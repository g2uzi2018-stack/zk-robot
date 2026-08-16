#pragma once

#include "tiago/can/can_config.hpp"

#include <filesystem>
#include <string>

namespace robot::tiago
{
    // 底盘配置，包括 CAN 接口、几何参数、速度限制和左右轮电机参数。
    struct BaseConfig
    {
        std::string interface_name;

        double wheel_radius{0.0};
        double wheel_separation{0.0};
        double max_linear_velocity{0.0};
        double max_angular_velocity{0.0};

        CanMotorConfig right_motor;
        CanMotorConfig left_motor;
    };

    // 从 YAML 文件加载并校验底盘配置。
    BaseConfig loadBaseConfig(const std::filesystem::path &config_path);
}
