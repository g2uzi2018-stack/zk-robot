#include "tiago/motor/can_motor.hpp"

#include "tiago/can/encoder_conversion.hpp"

#include <variant>

namespace robot::tiago
{
    // 保存当前电机配置和 CAN 总线引用。
    CanMotor::CanMotor(const CanMotorConfig &config, robot::can::SocketCan &can)
        : config_(config), can_(can)
    {
    }

    // 发送电机使能命令。
    void CanMotor::enable()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Enable);
        can_.send(frame);
    }

    // 发送电机禁用命令。
    void CanMotor::disable()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Disable);
        can_.send(frame);
    }

    // 清除当前电机的故障状态。
    void CanMotor::clearFault()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::ClearFault);
        can_.send(frame);
    }

    // 停止当前电机运动。
    void CanMotor::stop()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Stop);
        can_.send(frame);
    }

    // 从 CAN 总线上读取一帧属于当前电机的反馈。
    std::optional<MotorFeedback> CanMotor::readFeedback()
    {
        // 最多等待 10 ms。
        const auto frame = can_.receive(kFeedbackTimeout);
        if (!frame)
        {
            return std::nullopt;
        }

        if (!isFeedbackFrameId(frame->id))
        {
            return std::nullopt;
        }

        const auto feedback = decodeFeedbackFrame(*frame);
        if (feedback.node_id != config_.node_id)
        {
            return std::nullopt;
        }

        return feedback;
    }

    // 主动查询当前电机状态。
    std::optional<MotorFeedback> CanMotor::queryStatus()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::QueryStatus);
        can_.send(frame);
        return readFeedback();
    }

    // 发送位置控制命令。
    //
    // CanMotor 不处理机械限位，只负责物理量、编码器计数和 CAN 协议之间的转换。
    void CanMotor::commandPosition(double position, double velocity_limit)
    {
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

        const auto frame = encodePositionCommand(config_.node_id, position_counts, velocity_limit_counts);
        can_.send(frame);
    }

    // 读取当前电机位置。
    std::optional<double> CanMotor::readPosition()
    {
        const auto feedback = readFeedback();
        if (!feedback)
        {
            return std::nullopt;
        }

        if (config_.unit == JointUnit::Radian)
        {
            const auto &encoder = std::get<RotaryEncoderConfig>(config_.encoder);
            return countsToRadians(feedback->position_counts, encoder);
        }

        const auto &encoder = std::get<LinearEncoderConfig>(config_.encoder);
        return countsToMeters(feedback->position_counts, encoder);
    }
}
