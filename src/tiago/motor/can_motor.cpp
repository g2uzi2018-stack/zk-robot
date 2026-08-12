#include "tiago/motor/can_motor.hpp"

#include "tiago/can/encoder_conversion.hpp"

#include <variant>

namespace robot::tiago
{
    CanMotor::CanMotor(const CanMotorConfig &config, CanBus &bus)
        : config_(config),
          bus_(bus)
    {
    }

    // 发送电机使能命令。
    void CanMotor::enable()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Enable);

        bus_.send(frame);
    }

    // 发送电机禁用命令。
    void CanMotor::disable()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Disable);

        bus_.send(frame);
    }

    // 清除当前电机故障。
    void CanMotor::clearFault()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::ClearFault);

        bus_.send(frame);
    }

    // 停止当前电机。
    void CanMotor::stop()
    {
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::Stop);

        bus_.send(frame);
    }

    // 主动查询当前电机状态。
    std::optional<MotorFeedback> CanMotor::queryStatus()
    {
        // --------------------------------------------------------
        // 1. 先把 CAN socket 中之前积压的反馈全部取出。
        //
        // 这些反馈会被保存到 CanBus 的 latest_feedback_ 中，
        // 但不会作为这次 QueryStatus 的新反馈返回。
        // --------------------------------------------------------
        bus_.collectPendingFeedback();

        // --------------------------------------------------------
        // 2. 发送当前节点的 QueryStatus。
        // --------------------------------------------------------
        const auto frame = encodeControlCommand(config_.node_id, MotorControlCommand::QueryStatus);

        bus_.send(frame);

        // --------------------------------------------------------
        // 3. 等待当前 node_id 的新反馈。
        //
        // 如果过程中收到其他 node 的反馈，
        // CanBus 会保存下来，而不是直接丢掉。
        // --------------------------------------------------------
        return bus_.waitForFeedback(config_.node_id, kFeedbackTimeout);
    }

    // 发送位置控制命令。
    void CanMotor::commandPosition(double position, double velocity_limit)
    {
        std::int32_t position_counts{};

        std::uint16_t velocity_limit_counts{};

        // 旋转电机。
        if (config_.unit == JointUnit::Radian)
        {
            const auto &encoder = std::get<RotaryEncoderConfig>(config_.encoder);

            position_counts = radiansToCounts(position, encoder);

            velocity_limit_counts = radiansPerSecondToCountsPerSecond(velocity_limit, encoder);
        }

        // 直线电机。
        else
        {
            const auto &encoder = std::get<LinearEncoderConfig>(config_.encoder);

            position_counts = metersToCounts(position, encoder);

            velocity_limit_counts = metersPerSecondToCountsPerSecond(velocity_limit, encoder);
        }

        const auto frame = encodePositionCommand(
            config_.node_id, position_counts, velocity_limit_counts);

        bus_.send(frame);
    }

    std::optional<double> CanMotor::readPosition()
    {
        // 先把当前 CAN socket 中已经到达的反馈全部收进来。
        bus_.collectPendingFeedback();

        // 直接读取当前节点保存的最新状态。
        const auto feedback = bus_.latestFeedback(config_.node_id);

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
