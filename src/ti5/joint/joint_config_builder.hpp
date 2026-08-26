#pragma once

#include "ti5/config/config.hpp"
#include "ti5/joint/joint.hpp"

#include <vector>

namespace robot::ti5
{

// 将 robot.yaml 的物理电机、safety.yaml 的软件限位和可选的
// kinematics.yaml 模型坐标换算合并成 Joint 可以直接使用的配置。
//
// 不传 model 时为整机所有物理关节生成恒等坐标换算，适用于 Head 以及
// 电机坐标工具。传入 model 时只生成该模型声明的关节，并严格检查名称。
std::vector<JointConfig> makeJointConfigs(
    const Ti5RobotConfig &robot_config,
    const JointSafetyConfig &safety_config,
    const KinematicsModelConfig *model = nullptr);

} // namespace robot::ti5
