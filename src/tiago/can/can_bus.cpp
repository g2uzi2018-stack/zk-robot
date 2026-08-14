#include "tiago/can/can_bus.hpp"

#include <chrono>
#include <utility>

namespace robot::tiago
{
    CanBus::CanBus(std::string interface_name)
        : socket_(std::move(interface_name))
    {
    }

    // CAN 发送统一通过当前总线的 SocketCan 完成。
    void CanBus::send(const robot::can::CanFrame &frame)
    {
        socket_.send(frame);
    }

    // 如果是反馈帧，则解析后保存到对应 node_id。
    void CanBus::storeFeedback(const robot::can::CanFrame &frame)
    {
        // 非电机反馈帧不处理。
        if (!isFeedbackFrameId(frame.id))
        {
            return;
        }

        const auto feedback = decodeFeedbackFrame(frame);

        // map 的 operator[] 会：
        //
        //   没有这个 node_id -> 创建
        //   已经有这个 node_id -> 覆盖
        //
        // 因此始终只保存最新反馈。
        latest_feedback_[feedback.node_id] = feedback;
    }

    // 把当前 SocketCAN 中已经排队的所有帧取出来。
    //
    // timeout = 0 表示完全不等待新数据。
    void CanBus::collectPendingFeedback()
    {
        while (true)
        {
            const auto frame = socket_.receive(std::chrono::milliseconds{0});

            if (!frame)
            {
                break;
            }

            storeFeedback(*frame);
        }
    }

    // 等待指定 node_id 的新反馈。
    std::optional<MotorFeedback> CanBus::waitForFeedback(std::uint16_t node_id, std::chrono::milliseconds timeout)
    {
        const auto deadline =
            std::chrono::steady_clock::now() + timeout;

        while (true)
        {
            const auto now = std::chrono::steady_clock::now();

            if (now >= deadline)
            {
                return std::nullopt;
            }

            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);

            // duration_cast 可能把不足 1 ms 截断成 0。
            // 如果实际上还有剩余时间，至少等待 1 ms。
            if (remaining.count() == 0)
            {
                remaining = std::chrono::milliseconds{1};
            }

            const auto frame = socket_.receive(remaining);

            if (!frame)
            {
                return std::nullopt;
            }

            // 非反馈帧直接忽略。
            if (!isFeedbackFrameId(frame->id))
            {
                continue;
            }

            const auto feedback = decodeFeedbackFrame(*frame);

            // 无论是不是当前 Motor 的反馈，
            // 都保存到对应 node_id。
            latest_feedback_[feedback.node_id] = feedback;

            // 如果是我们正在等待的节点，
            // 再把当前已经排队的反馈全部吃完，
            // 保证 map 中尽量留下最新状态。
            if (feedback.node_id == node_id)
            {
                collectPendingFeedback();

                const auto latest = latest_feedback_.find(node_id);

                if (latest == latest_feedback_.end())
                {
                    return std::nullopt;
                }

                return latest->second;
            }
        }
    }

    // 读取 map 中某节点当前保存的最新反馈。
    std::optional<MotorFeedback> CanBus::latestFeedback(std::uint16_t node_id) const
    {
        const auto feedback = latest_feedback_.find(node_id);

        if (feedback == latest_feedback_.end())
        {
            return std::nullopt;
        }

        return feedback->second;
    }
}
