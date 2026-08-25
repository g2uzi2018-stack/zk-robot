#pragma once

#include "can/can_frame.hpp"
#include "can/can_receive_event.hpp"

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace robot::can
{
    struct SocketCanOptions
    {
        // 空列表表示接收全部标准数据帧；非空时只接收列出的 11-bit CAN ID。
        std::vector<std::uint16_t> accepted_ids;

        // 订阅 Linux SocketCAN 错误帧，用于识别错误警告、错误被动、
        // 总线关闭以及自动恢复事件。
        bool receive_error_frames{false};
    };

    // SocketCAN 设备的收发接口封装。
    class SocketCan
    {
    public:
        // 使用指定的 SocketCAN 接口创建对象，例如 vcan0 或 can0。
        explicit SocketCan(std::string interface_name,
                           SocketCanOptions options = {});

        // 关闭底层套接字并释放资源。
        ~SocketCan();

        // 发送一帧 CAN 数据。
        void send(const CanFrame &frame);

        // 在指定超时时间内接收一帧普通数据；超时则返回空值。
        // 需要处理 CAN 错误状态时使用 receiveEvent()。
        std::optional<CanFrame> receive(std::chrono::milliseconds timeout);

        // 接收普通数据帧或 CAN 错误事件；超时则返回空值。
        std::optional<CanReceiveEvent> receiveEvent(
            std::chrono::milliseconds timeout);

        // SocketCan 管理底层文件描述符，不允许复制。
        SocketCan(const SocketCan &) = delete;
        SocketCan &operator=(const SocketCan &) = delete;

        // 当前不提供移动语义。
        SocketCan(SocketCan &&) = delete;
        SocketCan &operator=(SocketCan &&) = delete;

    private:
        // SocketCAN 接口名称。
        std::string interface_name_;
        // 底层套接字文件描述符，-1 表示尚未打开。
        int socket_fd_{-1};
    };
}
