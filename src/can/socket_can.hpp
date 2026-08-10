#pragma once

#include "can/can_frame.hpp"

#include <chrono>
#include <optional>
#include <string>

namespace robot::can
{
    // SocketCAN 设备的收发接口封装。
    class SocketCan
    {
    public:
        // 使用指定的 SocketCAN 接口创建对象，例如 vcan0 或 can0。
        explicit SocketCan(std::string interface_name);

        // 关闭底层套接字并释放资源。
        ~SocketCan();

        // 发送一帧 CAN 数据。
        void send(const CanFrame &frame);

        // 在指定超时时间内接收一帧数据；超时则返回空值。
        std::optional<CanFrame> receive(std::chrono::milliseconds timeout);

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
