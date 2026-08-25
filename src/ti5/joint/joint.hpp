#pragma once

#include "ti5/config/config.hpp"
#include "ti5/motor/can_motor.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace robot::ti5
{

struct JointConfig
{
    PhysicalJointConfig physical_joint;
    JointPositionLimits motor_position_limits;
    JointCoordinateTransform coordinate_transform;
};

// 一个 T170C 物理关节的机器人语义封装。
//
// Joint 负责：
//   - 模型关节角与电机输出角之间的双向转换；
//   - safety.yaml 软件限位检查；
//   - 驱动器 0x1A/0x1B 目标范围检查；
//   - 将状态读取和 Position CSP 命令交给唯一的 CanMotor。
//
// Joint 不负责：
//   - CAN 收发、协议编码和总线恢复；
//   - 控制周期、速度限制、轨迹插值和多关节同步；
//   - 自然下垂接管、特殊边界恢复或共享轴所有权仲裁。
class Joint final
{
public:
    Joint(const JointConfig &config, CanBus &bus);

    const std::string &name() const noexcept;
    const std::string &physicalName() const noexcept;
    const std::string &busName() const noexcept;
    std::uint16_t nodeId() const noexcept;

    double motorToJointPosition(double motor_position_rad) const;
    double jointToMotorPosition(double joint_position_rad) const;

    const JointPositionLimits &motorPositionLimits() const noexcept;
    const JointPositionLimits &positionLimits() const noexcept;

    // 查询并缓存驱动器允许接受的电机角目标范围。未成功完成该查询前，
    // commandPositionCsp 会拒绝发送运动目标。
    std::optional<DriverPositionLimits> refreshDriverPositionLimits();
    std::optional<DriverPositionLimits> driverPositionLimits() const noexcept;

    // 返回主机软件限位与驱动器目标范围的交集，并转换为关节坐标。
    // 驱动器范围尚未加载或两者没有交集时返回空。
    std::optional<JointPositionLimits> positionCommandLimits() const;

    void validatePositionCommand(double joint_position_rad) const;
    void commandPositionCsp(double joint_position_rad);

    std::optional<double> queryPosition();
    std::optional<double> readPosition();
    std::optional<double> readVelocity();
    std::optional<double> readCurrentAmps();
    // 返回未做关节坐标换算的电机 CSP 原始反馈，供启动核对使用。
    std::optional<CspFeedback> queryMotorCspStatus();
    std::optional<DriverStatus> queryDriverStatus();
    std::optional<MotorState> latestMotorState();
    bool hasFreshCspFeedback(std::chrono::milliseconds maximum_age);

private:
    std::string name_;
    std::string physical_name_;
    std::string bus_name_;
    JointPositionLimits motor_position_limits_;
    JointPositionLimits joint_position_limits_;
    JointCoordinateTransform coordinate_transform_;
    std::optional<DriverPositionLimits> driver_position_limits_;
    CanMotor motor_;
};

} // namespace robot::ti5
