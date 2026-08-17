#pragma once

#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/joint/joint.hpp"

#include <optional>

namespace robot::tiago
{
    // TIAGo 躯干升降机构。
    //
    // Torso 负责：
    //   - 管理 torso 使用的一条 CAN 总线
    //   - 管理唯一的 torso_lift_joint
    //   - 提供基础位置控制接口
    //
    // Torso 不负责：
    //   - YAML 解析
    //   - CAN 协议和 SocketCAN 细节
    //   - 控制周期和线程
    //   - 上层升降策略
    class Torso
    {
    public:
        // 配置必须只包含一个 meter Joint。
        explicit Torso(const CanBusConfig &config);

        // 使能躯干升降电机。
        void enable();

        // 禁用躯干升降电机。
        void disable();

        // 清除躯干升降电机故障。
        void clearFault();

        // 停止躯干升降运动。
        void stop();

        // position 单位 m。
        // velocity_limit 单位 m/s。
        void commandPosition(double position, double velocity_limit);

        // 读取最新位置，单位 m。
        std::optional<double> readPosition();

        // 访问唯一的 torso_lift_joint。
        Joint &joint();
        const Joint &joint() const;

    private:
        static const JointConfig &validateConfig(const CanBusConfig &config);

        // 必须先于 joint_ 构造。
        CanBus bus_;

        Joint joint_;
    };
}
