#pragma once

#include "can/socket_can.hpp"
#include "tiago/can/can_protocol.hpp"

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace robot::tiago
{
    // 一条 CAN 总线的运行时对象。
    //
    // CanBus 负责管理共享的 SocketCAN 接口：
    //
    //   SocketCan
    //       ↓
    //   receive CAN frame
    //       ↓
    //   decode feedback
    //       ↓
    //   根据 node_id 保存每个电机的最新反馈
    //
    // 同一条 CAN 总线上的多个 CanMotor 共享同一个 CanBus。
    class CanBus
    {
    public:
        // 打开指定的 SocketCAN 接口。
        // 例如：
        //   vcan0
        //   can0
        explicit CanBus(std::string interface_name);

        // 向当前 CAN 总线发送一帧数据。
        void send(const robot::can::CanFrame &frame);

        // 读取当前 SocketCAN 接收队列中已经存在的所有反馈帧。
        //
        // 不等待新数据。
        //
        // 每个 node_id 只保存最新的一帧反馈，
        // 因此旧反馈会被新的反馈覆盖。
        void collectPendingFeedback();

        // 等待指定 CAN 节点的新反馈。
        //
        // 等待过程中如果收到其他节点的反馈，
        // 不会丢弃，而是保存到对应 node_id 的最新反馈中。
        //
        // 收到目标节点反馈后返回该节点最新状态。
        // 超时则返回 std::nullopt。
        std::optional<MotorFeedback> waitForFeedback(
            std::uint16_t node_id, std::chrono::milliseconds timeout);

        // 获取某个节点当前已经保存的最新反馈。
        //
        // 该函数不会访问 SocketCAN，
        // 只是读取 CanBus 内部保存的状态。
        std::optional<MotorFeedback> latestFeedback(std::uint16_t node_id) const;

    private:
        // 处理一帧 CAN 数据。
        //
        // 如果是电机反馈帧，则解析并按照 node_id
        // 更新 latest_feedback_。
        void storeFeedback(const robot::can::CanFrame &frame);

        // 当前 CAN 总线对应的 Linux SocketCAN 对象。
        robot::can::SocketCan socket_;

        // 每个 CAN 节点只保存最新的一帧反馈。
        //
        // key:
        //   node_id
        //
        // value:
        //   当前节点最新的 MotorFeedback
        std::map<std::uint16_t, MotorFeedback> latest_feedback_;
    };
}
