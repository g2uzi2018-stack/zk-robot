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
    // TIAGo 两自由度头部。
    //
    // Head 负责：
    //   - 管理头部使用的一条 CAN 总线
    //   - 管理两个旋转 Joint
    //   - 提供头部级别的位置控制接口
    //
    // Head 不负责：
    //   - YAML 解析
    //   - CAN 协议和 SocketCAN 细节
    //   - 控制周期和线程
    //   - 轨迹规划
    class Head
    {
    public:
        static constexpr std::size_t kJointCount = 2;

        // 顺序固定：
        //   index 0 -> head_1_joint
        //   index 1 -> head_2_joint
        using JointValues = std::array<double, kJointCount>;

        // 某个头部关节可能暂时没有反馈。
        using JointPositions = std::array<std::optional<double>, kJointCount>;

        // 配置必须包含两个旋转 Joint。
        explicit Head(const CanBusConfig &config);

        // 使能两个头部关节。
        void enable();

        // 禁用两个头部关节。
        void disable();

        // 清除两个头部关节的电机故障。
        void clearFault();

        // 停止两个头部关节运动。
        void stop();

        // 同时控制两个头部 Joint。
        //
        // positions:
        //   目标位置，单位 rad。
        //
        // velocity_limits:
        //   最大速度，单位 rad/s。
        //
        // 会先检查两个 Joint 的完整命令，
        // 全部合法后再真正发送。
        void commandPositions(const JointValues &positions, const JointValues &velocity_limits);

        // 读取两个 Joint 的最新位置。
        JointPositions readPositions();

        // 按固定顺序访问指定头部关节。
        Joint &joint(std::size_t index);
        const Joint &joint(std::size_t index) const;

    private:
        // Head 独占一条 CAN 总线。
        // 必须在 joints_ 之前声明。
        CanBus bus_;

        // index 0 -> head_1_joint
        // index 1 -> head_2_joint
        std::vector<Joint> joints_;
    };
}
