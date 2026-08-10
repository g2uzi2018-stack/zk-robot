#include "can/socket_can.hpp"

#include <algorithm>
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <stdexcept>
#include <system_error>
#include <sys/socket.h>
#include <unistd.h>
#include <utility>

namespace
{
    // 将系统调用错误转换为包含接口名称的异常。
    [[noreturn]] void throwSocketCanError(int error_number, const std::string &operation, const std::string &interface_name)
    {
        throw std::system_error(error_number, std::generic_category(), operation + " failed for CAN interface '" + interface_name + "'");
    }
}

namespace robot::can
{
    // 打开并绑定指定的 SocketCAN 接口。
    SocketCan::SocketCan(std::string interface_name)
        : interface_name_(std::move(interface_name))
    {
        if (interface_name_.empty())
        {
            throw std::invalid_argument("CAN interface name must not be empty");
        }

        const unsigned int interface_index = if_nametoindex(interface_name_.c_str());

        if (interface_index == 0)
        {
            const int error_number = errno != 0 ? errno : ENODEV;
            throwSocketCanError(error_number, "Resolve CAN interface", interface_name_);
        }

        // 使用非阻塞原始 CAN 套接字，接收超时由 poll 控制。
        const int socket_fd = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, CAN_RAW);

        if (socket_fd < 0)
        {
            throwSocketCanError(errno, "Create SocketCAN socket", interface_name_);
        }

        sockaddr_can address{};
        address.can_family = AF_CAN;
        address.can_ifindex = static_cast<int>(interface_index);

        if (::bind(socket_fd, reinterpret_cast<const sockaddr *>(&address), sizeof(address)) < 0)
        {
            const int error_number = errno;
            ::close(socket_fd);
            throwSocketCanError(error_number, "Bind SocketCAN socket", interface_name_);
        }

        socket_fd_ = socket_fd;
    }

    // 关闭底层 SocketCAN 文件描述符。
    SocketCan::~SocketCan()
    {
        if (socket_fd_ >= 0)
        {
            ::close(socket_fd_);
        }
    }

    void SocketCan::send(const CanFrame &frame)
    {
        if (frame.id > CAN_SFF_MASK)
        {
            throw std::invalid_argument("CAN frame ID exceeds 11-bit range");
        }

        if (frame.data_length > CAN_MAX_DLEN)
        {
            throw std::invalid_argument("CAN frame data length exceeds 8 bytes");
        }

        // 将项目内的 CanFrame 转换为 Linux SocketCAN 帧结构。
        struct can_frame native_frame{};
        native_frame.can_id = static_cast<canid_t>(frame.id);
        native_frame.can_dlc = frame.data_length;
        std::copy_n(frame.data.begin(), frame.data_length, native_frame.data);
        const ssize_t bytes_written = ::write(socket_fd_, &native_frame, sizeof(native_frame));

        if (bytes_written < 0)
        {
            throwSocketCanError(errno, "Write CAN frame", interface_name_);
        }

        if (bytes_written != static_cast<ssize_t>(sizeof(native_frame)))
        {
            throwSocketCanError(EIO, "Write complete CAN frame", interface_name_);
        }
    }

    // 在指定超时时间内接收一帧 CAN 数据。
    std::optional<CanFrame> SocketCan::receive(std::chrono::milliseconds timeout)
    {
        if (timeout.count() < 0)
        {
            throw std::invalid_argument("CAN receive timeout must not be negative");
        }

        pollfd descriptor{};
        descriptor.fd = socket_fd_;
        descriptor.events = POLLIN;
        int poll_result{};

        // 被信号中断时重新等待，直到超时、收到数据或发生错误。
        do
        {
            poll_result = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
        }
        while (poll_result < 0 && errno == EINTR);

        if (poll_result == 0)
        {
            return std::nullopt;
        }

        if (poll_result < 0)
        {
            throwSocketCanError(errno, "Wait for CAN frame", interface_name_);
        }

        if ((descriptor.revents & POLLNVAL) != 0)
        {
            throwSocketCanError(EBADF, "Poll CAN socket", interface_name_);
        }

        if ((descriptor.revents & (POLLERR | POLLHUP)) != 0)
        {
            throwSocketCanError(EIO, "Poll CAN socket", interface_name_);
        }

        // 从底层套接字读取一帧原始 CAN 数据。
        struct can_frame native_frame{};
        const ssize_t bytes_read = ::read(socket_fd_, &native_frame, sizeof(native_frame));

        if (bytes_read < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                return std::nullopt;
            }

            throwSocketCanError(errno, "Read CAN frame", interface_name_);
        }

        if (bytes_read != static_cast<ssize_t>(sizeof(native_frame)))
        {
            throwSocketCanError(EIO, "Read complete CAN frame", interface_name_);
        }

        // 当前封装只接受标准数据帧，不支持扩展帧、远程帧和错误帧。
        constexpr canid_t unsupported_flags = CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG;

        if ((native_frame.can_id & unsupported_flags) != 0)
        {
            throw std::runtime_error("Received unsupported CAN frame type");
        }

        if (native_frame.can_dlc > CAN_MAX_DLEN)
        {
            throw std::runtime_error("Received CAN frame length exceeds 8 bytes");
        }

        // 将 Linux 帧转换为项目内的 CanFrame 类型。
        CanFrame frame{};
        frame.id = static_cast<std::uint16_t>(native_frame.can_id & CAN_SFF_MASK);
        frame.data_length = native_frame.can_dlc;
        std::copy_n(native_frame.data, frame.data_length, frame.data.begin());
        return frame;
    }
}
