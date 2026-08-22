#include "can/can_interface_manager.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <linux/can/netlink.h>
#include <linux/if_link.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <regex>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace
{

using robot::can::CanInterfaceInfo;

class NetlinkSocket final
{
public:
    NetlinkSocket()
    {
        fd_ = ::socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE);
        if (fd_ < 0)
        {
            throw std::runtime_error(
                "创建 NETLINK_ROUTE socket 失败: " +
                std::string(std::strerror(errno)));
        }

        sockaddr_nl address{};
        address.nl_family = AF_NETLINK;
        if (::bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0)
        {
            const int saved_errno = errno;
            ::close(fd_);
            fd_ = -1;
            throw std::runtime_error(
                "绑定 NETLINK_ROUTE socket 失败: " +
                std::string(std::strerror(saved_errno)));
        }
    }

    ~NetlinkSocket()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
        }
    }

    int fd() const noexcept
    {
        return fd_;
    }

private:
    int fd_{-1};
};

[[noreturn]] void throwNetlinkError(const std::string &operation,
                                    const int error_code)
{
    if (error_code == EPERM || error_code == EACCES)
    {
        throw std::runtime_error(
            operation + " 失败：缺少 CAP_NET_ADMIN（或需要 root 权限）");
    }

    throw std::runtime_error(
        operation + " 失败: " + std::string(std::strerror(error_code)));
}

void sendMessage(const int fd, const nlmsghdr *header)
{
    sockaddr_nl kernel{};
    kernel.nl_family = AF_NETLINK;

    iovec iov{
        const_cast<nlmsghdr *>(header),
        header->nlmsg_len};

    msghdr message{};
    message.msg_name = &kernel;
    message.msg_namelen = sizeof(kernel);
    message.msg_iov = &iov;
    message.msg_iovlen = 1;

    if (::sendmsg(fd, &message, 0) < 0)
    {
        throwNetlinkError("发送 rtnetlink 请求", errno);
    }
}

void waitForAck(const int fd,
                const std::uint32_t sequence,
                const std::string &operation)
{
    std::array<char, 8192> buffer{};

    while (true)
    {
        const auto received = ::recv(fd, buffer.data(), buffer.size(), 0);
        if (received < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throwNetlinkError(operation, errno);
        }

        int remaining = static_cast<int>(received);
        for (auto *header = reinterpret_cast<nlmsghdr *>(buffer.data());
             NLMSG_OK(header, remaining);
             header = NLMSG_NEXT(header, remaining))
        {
            if (header->nlmsg_seq != sequence ||
                header->nlmsg_type != NLMSG_ERROR)
            {
                continue;
            }

            const auto *error = reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(header));
            if (error->error == 0)
            {
                return;
            }
            throwNetlinkError(operation, -error->error);
        }
    }
}

template <std::size_t N>
void addAttribute(nlmsghdr &header,
                  std::array<char, N> &storage,
                  const std::uint16_t type,
                  const void *data,
                  const std::size_t length)
{
    const std::size_t aligned = NLMSG_ALIGN(header.nlmsg_len);
    const std::size_t attribute_length = RTA_LENGTH(length);
    const std::size_t next = aligned + RTA_ALIGN(attribute_length);

    const auto base = reinterpret_cast<std::uintptr_t>(&header);
    const auto storage_end = reinterpret_cast<std::uintptr_t>(storage.data() + storage.size());
    if (base + next > storage_end)
    {
        throw std::runtime_error("rtnetlink 请求缓冲区不足");
    }

    auto *attribute = reinterpret_cast<rtattr *>(
        reinterpret_cast<char *>(&header) + aligned);
    attribute->rta_type = type;
    attribute->rta_len = static_cast<unsigned short>(attribute_length);
    if (length > 0)
    {
        std::memcpy(RTA_DATA(attribute), data, length);
    }
    header.nlmsg_len = static_cast<std::uint32_t>(next);
}

template <std::size_t N>
rtattr *beginNestedAttribute(nlmsghdr &header,
                             std::array<char, N> &storage,
                             const std::uint16_t type)
{
    const std::size_t offset = NLMSG_ALIGN(header.nlmsg_len);
    addAttribute(header, storage, type, nullptr, 0);
    return reinterpret_cast<rtattr *>(
        reinterpret_cast<char *>(&header) + offset);
}

void endNestedAttribute(nlmsghdr &header, rtattr *attribute)
{
    attribute->rta_len = static_cast<unsigned short>(
        reinterpret_cast<char *>(&header) + header.nlmsg_len -
        reinterpret_cast<char *>(attribute));
}

std::optional<std::string> readTextFile(const std::filesystem::path &path)
{
    std::ifstream input(path);
    if (!input)
    {
        return std::nullopt;
    }

    std::string value;
    std::getline(input, value);
    while (!value.empty() &&
           (value.back() == '\n' || value.back() == '\r' ||
            value.back() == ' ' || value.back() == '\t'))
    {
        value.pop_back();
    }
    if (value.empty())
    {
        return std::nullopt;
    }
    return value;
}

std::optional<std::string> readUdevProperty(const unsigned int ifindex,
                                            const std::string &key)
{
    const auto path = std::filesystem::path("/run/udev/data") /
                      ("n" + std::to_string(ifindex));
    std::ifstream input(path);
    if (!input)
    {
        return std::nullopt;
    }

    const std::string prefix = "E:" + key + "=";
    std::string line;
    while (std::getline(input, line))
    {
        if (line.rfind(prefix, 0) == 0)
        {
            const auto value = line.substr(prefix.size());
            if (!value.empty())
            {
                return value;
            }
        }
    }
    return std::nullopt;
}

void fillSysfsIdentity(CanInterfaceInfo &info)
{
    std::error_code error;
    const auto device_link = std::filesystem::path("/sys/class/net") /
                             info.name / "device";
    const auto device_path = std::filesystem::canonical(device_link, error);
    if (!error)
    {
        info.device_path = device_path.string();
        info.sysfs_parent = device_path.filename().string();
        info.usb_parent = info.sysfs_parent;

        auto current = device_path;
        while (!current.empty() && current != current.root_path())
        {
            if (const auto serial = readTextFile(current / "serial"))
            {
                info.usb_serial = *serial;
                info.usb_serial_short = *serial;
                break;
            }
            current = current.parent_path();
        }
    }

    if (const auto value = readUdevProperty(info.ifindex, "ID_SERIAL_SHORT"))
    {
        info.usb_serial_short = *value;
    }
    if (const auto value = readUdevProperty(info.ifindex, "ID_SERIAL"))
    {
        info.id_serial = *value;
        if (!info.usb_serial)
        {
            info.usb_serial = *value;
        }
    }
    if (const auto value = readUdevProperty(info.ifindex, "ID_PATH"))
    {
        info.id_path = *value;
    }
}

CanInterfaceInfo queryLinkInfo(const std::string &interface_name)
{
    const unsigned int ifindex = ::if_nametoindex(interface_name.c_str());
    if (ifindex == 0)
    {
        throw std::invalid_argument("CAN 接口不存在: " + interface_name);
    }

    struct Request
    {
        nlmsghdr header;
        ifinfomsg info;
    };

    Request request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_GETLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST;
    request.header.nlmsg_seq = 1;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);

    NetlinkSocket socket;
    sendMessage(socket.fd(), &request.header);

    std::array<char, 16384> buffer{};
    while (true)
    {
        const auto received = ::recv(socket.fd(), buffer.data(), buffer.size(), 0);
        if (received < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            throwNetlinkError("读取 CAN 接口状态", errno);
        }

        int remaining = static_cast<int>(received);
        for (auto *header = reinterpret_cast<nlmsghdr *>(buffer.data());
             NLMSG_OK(header, remaining);
             header = NLMSG_NEXT(header, remaining))
        {
            if (header->nlmsg_seq != 1)
            {
                continue;
            }
            if (header->nlmsg_type == NLMSG_ERROR)
            {
                const auto *error = reinterpret_cast<const nlmsgerr *>(NLMSG_DATA(header));
                if (error->error != 0)
                {
                    throwNetlinkError("读取 CAN 接口状态", -error->error);
                }
                continue;
            }
            if (header->nlmsg_type != RTM_NEWLINK)
            {
                continue;
            }

            const auto *interface_info = reinterpret_cast<const ifinfomsg *>(NLMSG_DATA(header));
            if (static_cast<unsigned int>(interface_info->ifi_index) != ifindex)
            {
                continue;
            }

            CanInterfaceInfo result;
            result.name = interface_name;
            result.ifindex = ifindex;
            result.up = (interface_info->ifi_flags & IFF_UP) != 0;

            bool is_can = false;
            int attribute_length = IFLA_PAYLOAD(header);
            for (auto *attribute = IFLA_RTA(const_cast<ifinfomsg *>(interface_info));
                 RTA_OK(attribute, attribute_length);
                 attribute = RTA_NEXT(attribute, attribute_length))
            {
                if ((attribute->rta_type & NLA_TYPE_MASK) != IFLA_LINKINFO)
                {
                    continue;
                }

                std::string kind;
                rtattr *info_data = nullptr;
                int link_length = RTA_PAYLOAD(attribute);
                for (auto *link_attribute = reinterpret_cast<rtattr *>(RTA_DATA(attribute));
                     RTA_OK(link_attribute, link_length);
                     link_attribute = RTA_NEXT(link_attribute, link_length))
                {
                    const auto type = link_attribute->rta_type & NLA_TYPE_MASK;
                    if (type == IFLA_INFO_KIND)
                    {
                        kind.assign(static_cast<const char *>(RTA_DATA(link_attribute)),
                                    RTA_PAYLOAD(link_attribute));
                        while (!kind.empty() && kind.back() == '\0')
                        {
                            kind.pop_back();
                        }
                    }
                    else if (type == IFLA_INFO_DATA)
                    {
                        info_data = link_attribute;
                    }
                }

                if (kind != "can" || info_data == nullptr)
                {
                    continue;
                }
                is_can = true;

                int can_length = RTA_PAYLOAD(info_data);
                for (auto *can_attribute = reinterpret_cast<rtattr *>(RTA_DATA(info_data));
                     RTA_OK(can_attribute, can_length);
                     can_attribute = RTA_NEXT(can_attribute, can_length))
                {
                    const auto type = can_attribute->rta_type & NLA_TYPE_MASK;
                    if (type == IFLA_CAN_BITTIMING &&
                        RTA_PAYLOAD(can_attribute) >= sizeof(can_bittiming))
                    {
                        can_bittiming timing{};
                        std::memcpy(&timing, RTA_DATA(can_attribute), sizeof(timing));
                        if (timing.bitrate != 0)
                        {
                            result.bitrate = timing.bitrate;
                        }
                    }
                    else if (type == IFLA_CAN_RESTART_MS &&
                             RTA_PAYLOAD(can_attribute) >= sizeof(std::uint32_t))
                    {
                        std::uint32_t restart_ms{};
                        std::memcpy(&restart_ms, RTA_DATA(can_attribute), sizeof(restart_ms));
                        result.restart_ms = restart_ms;
                    }
                }
            }

            if (!is_can)
            {
                throw std::invalid_argument(interface_name + " 不是 CAN 接口");
            }
            fillSysfsIdentity(result);
            return result;
        }
    }
}

void setLinkUp(const std::string &interface_name,
               const unsigned int ifindex,
               const bool up)
{
    struct Request
    {
        nlmsghdr header;
        ifinfomsg info;
    };

    Request request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_NEWLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    request.header.nlmsg_seq = 1;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);
    request.info.ifi_change = IFF_UP;
    request.info.ifi_flags = up ? IFF_UP : 0;

    NetlinkSocket socket;
    sendMessage(socket.fd(), &request.header);
    waitForAck(socket.fd(), 1, (up ? "启动 " : "停止 ") + interface_name);
}

void configureCan(const unsigned int ifindex,
                  const std::string &interface_name,
                  const std::uint32_t bitrate,
                  const std::uint32_t restart_ms)
{
    struct Request
    {
        nlmsghdr header;
        ifinfomsg info;
        std::array<char, 1024> attributes;
    };

    Request request{};
    request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
    request.header.nlmsg_type = RTM_NEWLINK;
    request.header.nlmsg_flags = NLM_F_REQUEST | NLM_F_ACK;
    request.header.nlmsg_seq = 1;
    request.info.ifi_family = AF_UNSPEC;
    request.info.ifi_index = static_cast<int>(ifindex);

    auto *link_info = beginNestedAttribute(
        request.header,
        request.attributes,
        static_cast<std::uint16_t>(IFLA_LINKINFO | NLA_F_NESTED));

    constexpr char kind[] = "can";
    addAttribute(request.header,
                 request.attributes,
                 IFLA_INFO_KIND,
                 kind,
                 sizeof(kind));

    auto *info_data = beginNestedAttribute(
        request.header,
        request.attributes,
        static_cast<std::uint16_t>(IFLA_INFO_DATA | NLA_F_NESTED));

    can_bittiming timing{};
    timing.bitrate = bitrate;
    addAttribute(request.header,
                 request.attributes,
                 IFLA_CAN_BITTIMING,
                 &timing,
                 sizeof(timing));
    addAttribute(request.header,
                 request.attributes,
                 IFLA_CAN_RESTART_MS,
                 &restart_ms,
                 sizeof(restart_ms));

    endNestedAttribute(request.header, info_data);
    endNestedAttribute(request.header, link_info);

    NetlinkSocket socket;
    sendMessage(socket.fd(), &request.header);
    waitForAck(socket.fd(), 1, "配置 " + interface_name);
}

bool matchesSelector(const CanInterfaceInfo &info,
                     const robot::can::CanAdapterSelector &selector)
{
    if (selector.kind == "usb_serial_short")
    {
        return info.usb_serial_short && *info.usb_serial_short == selector.value;
    }
    if (selector.kind == "usb_serial")
    {
        return (info.usb_serial && *info.usb_serial == selector.value) ||
               (info.id_serial && *info.id_serial == selector.value);
    }
    if (selector.kind == "id_path")
    {
        return info.id_path && *info.id_path == selector.value;
    }
    if (selector.kind == "sysfs_parent")
    {
        return info.sysfs_parent == selector.value || info.usb_parent == selector.value;
    }
    if (selector.kind == "device_path")
    {
        return info.device_path == selector.value;
    }
    return false;
}

} // namespace

namespace robot::can
{

std::vector<CanInterfaceInfo> CanInterfaceManager::enumerate(
    const std::string &interface_regex) const
{
    std::regex pattern;
    try
    {
        pattern = std::regex(interface_regex);
    }
    catch (const std::regex_error &error)
    {
        throw std::invalid_argument(
            "CAN interface 正则表达式无效: " + std::string(error.what()));
    }

    struct if_nameindex *interfaces = ::if_nameindex();
    if (interfaces == nullptr)
    {
        throw std::runtime_error(
            "枚举网络接口失败: " + std::string(std::strerror(errno)));
    }

    std::vector<CanInterfaceInfo> result;
    try
    {
        for (auto *entry = interfaces; entry->if_index != 0; ++entry)
        {
            if (entry->if_name == nullptr)
            {
                continue;
            }
            const std::string name{entry->if_name};
            if (std::regex_match(name, pattern))
            {
                result.push_back(inspect(name));
            }
        }
    }
    catch (...)
    {
        ::if_freenameindex(interfaces);
        throw;
    }
    ::if_freenameindex(interfaces);

    std::sort(result.begin(), result.end(), [](const auto &left, const auto &right) {
        if (left.ifindex != right.ifindex)
        {
            return left.ifindex < right.ifindex;
        }
        return left.name < right.name;
    });
    return result;
}

CanInterfaceInfo CanInterfaceManager::inspect(const std::string &interface_name) const
{
    return queryLinkInfo(interface_name);
}

std::vector<CanInterfaceInfo> CanInterfaceManager::selectAdapter(
    const std::vector<CanInterfaceInfo> &interfaces,
    const CanAdapterSelector &selector) const
{
    if (selector.kind != "usb_serial_short" &&
        selector.kind != "usb_serial" &&
        selector.kind != "id_path" &&
        selector.kind != "sysfs_parent" &&
        selector.kind != "device_path")
    {
        throw std::invalid_argument("不支持的 CAN 适配器 selector: " + selector.kind);
    }
    if (selector.value.empty())
    {
        throw std::invalid_argument("CAN 适配器 selector value 不能为空");
    }

    std::vector<CanInterfaceInfo> selected;
    for (const auto &interface : interfaces)
    {
        if (matchesSelector(interface, selector))
        {
            selected.push_back(interface);
        }
    }

    if (selected.empty())
    {
        throw std::runtime_error(
            "找不到匹配 CAN 适配器的接口: " + selector.kind + "=" + selector.value);
    }
    if (selector.expected_channels != 0 &&
        selected.size() != selector.expected_channels)
    {
        throw std::runtime_error(
            "CAN 适配器通道数不符合预期: selector=" + selector.kind +
            "=" + selector.value + ", expected=" +
            std::to_string(selector.expected_channels) + ", actual=" +
            std::to_string(selected.size()));
    }

    std::sort(selected.begin(), selected.end(), [](const auto &left, const auto &right) {
        if (left.ifindex != right.ifindex)
        {
            return left.ifindex < right.ifindex;
        }
        return left.name < right.name;
    });
    return selected;
}

CanInterfaceInfo CanInterfaceManager::prepare(
    const std::string &interface_name,
    const CanInterfaceSettings &settings) const
{
    if (settings.bitrate == 0)
    {
        throw std::invalid_argument("CAN bitrate 必须大于 0");
    }

    const auto current = inspect(interface_name);
    const bool bitrate_ok =
        !settings.validate_bitrate ||
        (current.bitrate && *current.bitrate == settings.bitrate);
    const bool restart_ok =
        current.restart_ms && *current.restart_ms == settings.restart_ms;

    if (current.up && bitrate_ok && restart_ok)
    {
        return current;
    }

    if (current.up)
    {
        setLinkUp(interface_name, current.ifindex, false);
        if (settings.reconfigure_wait.count() > 0)
        {
            std::this_thread::sleep_for(settings.reconfigure_wait);
        }
    }

    configureCan(current.ifindex, interface_name, settings.bitrate, settings.restart_ms);
    setLinkUp(interface_name, current.ifindex, true);
    if (settings.startup_wait.count() > 0)
    {
        std::this_thread::sleep_for(settings.startup_wait);
    }

    const auto verified = inspect(interface_name);
    if (!verified.up)
    {
        throw std::runtime_error(interface_name + " 配置后仍未处于 UP 状态");
    }
    if (settings.validate_bitrate &&
        (!verified.bitrate || *verified.bitrate != settings.bitrate))
    {
        throw std::runtime_error(interface_name + " 配置后 bitrate 校验失败");
    }
    if (!verified.restart_ms || *verified.restart_ms != settings.restart_ms)
    {
        throw std::runtime_error(interface_name + " 配置后 restart-ms 校验失败");
    }
    return verified;
}

} // namespace robot::can
