#pragma once

#include <array>
#include <cstdint>

namespace robot::can
{
    struct CanFrame
    {
        // Classical CAN 标准帧 ID：0 ~ 0x7FF
        std::uint16_t id{0};

        // 有效数据长度：0 ~ 8
        std::uint8_t data_length{0};

        // CAN 数据区
        std::array<std::uint8_t, 8> data{};
    };
}