#include "tiago/motor/can_motor.hpp"

#include "tiago/can/can_protocol.hpp"
#include "tiago/can/encoder_conversion.hpp"

#include <chrono>
#include <stdexcept>
#include <variant>

namespace robot::tiago
{
    // 保存关节配置和 CAN 总线引用，后续命令与反馈均通过该总线处理。
    CanJoint::CanJoint(const CanJointConfig &config, robot::can::SocketCan &can)
        : config_(config), can_(can)
    {
    }

    // 确保目标位置没有超出关节配置的机械限位。
    void CanJoint::validateTargetPosition(double position) const
    {
        if (position < config_.limits.min_position || position > config_.limits.max_position)
        {
            throw std::out_of_range("Target position exceeds joint limits");
        }
    }

    // 确保速度上限为非负值，且不超过配置中的最大速度。
    void CanJoint::validateVelocityLimit(double velocity_limit) const
    {
        if (velocity_limit < 0.0)
        {
            throw std::out_of_range("Velocity limit must not be negative");
        }
        if (velocity_limit > config_.limits.max_velocity)
        {
            throw std::out_of_range("Velocity limit exceeds joint max_velocity");
        }
    }

    // 校验并转换目标位置和速度上限，然后发送位置控制帧。
    void CanJoint::commandPosition(double position, double velocity_limit)
    {
        validateTargetPosition(position);
        validateVelocityLimit(velocity_limit);

        std::int32_t position_counts{};
        std::uint16_t velocity_limit_counts{};

        if (config_.unit == JointUnit::Radian)
        {
            const auto &encoder = std::get<RotaryEncoderConfig>(config_.encoder);
            position_counts = radiansToCounts(position, encoder);
            velocity_limit_counts = radiansPerSecondToCountsPerSecond(velocity_limit, encoder);
        }
        else
        {
            const auto &encoder = std::get<LinearEncoderConfig>(config_.encoder);
            position_counts = metersToCounts(position, encoder);
            velocity_limit_counts = metersPerSecondToCountsPerSecond(velocity_limit, encoder);
        }

        // 根据节点 ID 编码位置命令，并通过 SocketCAN 发送。
        const auto frame = encodePositionCommand(config_.node_id, position_counts, velocity_limit_counts);
        can_.send(frame);
    }

    // 接收并解析一帧反馈，只返回当前关节对应的物理位置。
    std::optional<double> CanJoint::readPosition()
    {
        // 最多等待 10 毫秒，避免读取操作长时间阻塞。
        const auto frame = can_.receive(std::chrono::milliseconds{10});

        // 超时未收到数据。
        if (!frame)
        {
            return std::nullopt;
        }

        // 忽略非反馈帧。
        if (!isFeedbackFrameId(frame->id))
        {
            return std::nullopt;
        }

        const auto feedback = decodeFeedbackFrame(*frame);

        // 忽略属于其他 CAN 节点的反馈。
        if (feedback.node_id != config_.node_id)
        {
            return std::nullopt;
        }

        if (config_.unit == JointUnit::Radian)
        {
            const auto &encoder = std::get<RotaryEncoderConfig>(config_.encoder);
            return countsToRadians(feedback.position_counts, encoder);
        }
        else
        {
            const auto &encoder = std::get<LinearEncoderConfig>(config_.encoder);
            return countsToMeters(feedback.position_counts, encoder);
        }
    }

}
