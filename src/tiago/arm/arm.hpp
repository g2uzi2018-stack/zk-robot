#pragma once

#include "tiago/can/can_bus.hpp"
#include "tiago/can/can_config.hpp"
#include "tiago/joint/joint.hpp"

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

namespace robot::tiago
{
    // 一条 7 自由度机械臂。
    //
    // Arm 负责：
    //   - 管理机械臂使用的 3 条 CAN 总线
    //   - 管理 7 个 Joint
    //   - 提供机械臂级别的基础控制接口
    //
    // Arm 不负责：
    //   - YAML 解析
    //   - CAN 协议和 SocketCAN 细节
    //   - 单关节机械限位
    //   - 控制周期、轨迹插值和运动学
    class Arm
    {
    public:
        static constexpr std::size_t kJointCount = 7;

        // 7 个关节的一组数值。
        // 顺序固定对应 joint 1 ~ joint 7。
        using JointValues = std::array<double, kJointCount>;

        // 读取位置时某个 Joint 可能暂时没有反馈。
        using JointPositions = std::array<std::optional<double>, kJointCount>;

        // 使用 shoulder / elbow / wrist 三条 CAN 总线配置
        // 创建一条完整的 7 自由度机械臂。
        //
        // 配置中的关节顺序约定为：
        //
        // shoulder:
        //   joint 1, joint 2
        //
        // elbow:
        //   joint 3, joint 4
        //
        // wrist:
        //   joint 5, joint 6, joint 7
        Arm(const CanBusConfig &shoulder_config, const CanBusConfig &elbow_config, const CanBusConfig &wrist_config);

        // 使能整条机械臂。
        void enable();

        // 禁用整条机械臂。
        void disable();

        // 清除所有关节电机故障。
        void clearFault();

        // 停止所有关节运动。
        void stop();

        // 向 7 个关节发送位置命令。
        //
        // positions:
        //   joint 1 ~ joint 7 的目标位置。
        //
        // velocity_limits:
        //   joint 1 ~ joint 7 的本次速度限制。
        void commandPositions(const JointValues &positions, const JointValues &velocity_limits);

        // 读取 7 个关节当前保存的最新位置。
        JointPositions readPositions();

        // 按机械臂关节顺序访问指定 Joint。
        Joint &joint(std::size_t index);
        const Joint &joint(std::size_t index) const;

    private:
        // 三条 CAN 总线必须在 joints_ 之前声明，
        // 保证所有 Joint 内部引用的 CanBus 始终有效。
        CanBus shoulder_bus_;
        CanBus elbow_bus_;
        CanBus wrist_bus_;

        // 固定包含 joint 1 ~ joint 7。
        std::vector<Joint> joints_;
    };
}
