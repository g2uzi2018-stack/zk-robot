#pragma once

#include "can/socket_can.hpp"
#include "tiago/can/can_config.hpp"

#include <optional>

namespace robot::tiago
{
    // 将单个关节配置与 CAN 总线绑定，负责命令发送和反馈读取。
    class CanJoint
    {
    public:
        // 使用关节配置和已打开的 SocketCAN 总线创建对象。
        CanJoint(const CanJointConfig &config, robot::can::SocketCan &can);

        // 发送目标位置和速度上限，参数单位由关节配置决定。
        void commandPosition(double position, double velocity_limit);

        // 读取当前关节的位置反馈；未收到匹配反馈时返回空值。
        std::optional<double> readPosition();

    private:
        // 检查目标位置是否在配置的关节范围内。
        void validateTargetPosition(double position) const;

        // 检查速度上限是否在允许范围内。
        void validateVelocityLimit(double velocity_limit) const;

        // 当前关节的配置，包括节点 ID、单位、限位和编码器参数。
        CanJointConfig config_;

        // 由外部管理生命周期的 CAN 总线对象。
        robot::can::SocketCan &can_;
    };
}
