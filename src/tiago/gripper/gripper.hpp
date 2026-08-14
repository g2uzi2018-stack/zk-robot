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
    // 两指夹爪。
    //
    // Gripper 负责：
    //   - 管理夹爪使用的一条 CAN 总线
    //   - 管理两个独立的 finger Joint
    //   - 提供夹爪级别的基础控制接口
    //
    // Gripper 不负责：
    //   - YAML 解析
    //   - CAN 协议和 SocketCAN 细节
    //   - 控制周期和线程
    //   - 抓取力控制
    //   - 判断是否抓住物体
    //   - open / close 等上层抓取策略
    class Gripper
    {
    public:
        static constexpr std::size_t kFingerCount = 2;

        // 两个 finger 的一组数值。
        //
        // 顺序固定为：
        //   index 0 -> right finger
        //   index 1 -> left finger
        using FingerValues = std::array<double, kFingerCount>;

        // 某个 finger 当前可能暂时没有反馈。
        using FingerPositions = std::array<std::optional<double>, kFingerCount>;

        // 使用一条夹爪 CAN 总线配置创建 Gripper。
        //
        // 配置必须包含两个 Joint，顺序约定为：
        //
        //   joints[0] -> right finger
        //   joints[1] -> left finger
        explicit Gripper(const CanBusConfig &config);

        // 使能两个 finger。
        void enable();

        // 禁用两个 finger。
        void disable();

        // 清除两个 finger 电机故障。
        void clearFault();

        // 停止两个 finger 运动。
        void stop();

        // 向两个 finger 发送独立的位置命令。
        //
        // positions:
        //   两个 finger 的目标位置，单位为 m。
        //
        // velocity_limits:
        //   两个 finger 的速度限制，单位为 m/s。
        //
        // 会先检查两个 Joint 的完整命令，
        // 全部合法后再真正发送。
        void commandPositions(const FingerValues &positions, const FingerValues &velocity_limits);

        // 对称控制两个 finger。
        //
        // 两个 finger 使用相同的位置和速度限制。
        //
        // 注意：
        // finger_position 表示单个 finger 的关节位置，
        // 不是夹爪总开口宽度。
        void commandSymmetric(double finger_position, double velocity_limit);

        // 读取两个 finger 当前保存的最新位置。
        FingerPositions readPositions();

        // 按固定顺序访问指定 finger Joint。
        Joint &finger(std::size_t index);
        const Joint &finger(std::size_t index) const;

    private:
        // Gripper 拥有唯一的一条 CAN 总线。
        //
        // 必须在 fingers_ 之前声明，
        // 保证 Joint 内部引用的 CanBus 始终有效。
        CanBus bus_;

        // 固定包含：
        //   index 0 -> right finger
        //   index 1 -> left finger
        std::vector<Joint> fingers_;
    };
}
