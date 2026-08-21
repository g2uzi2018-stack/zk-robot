#pragma once

#include <cstdint>

namespace robot::ti5
{
    // T170C 双编码器当前使用的输出端一圈计数。
    constexpr std::uint32_t kT170cCountsPerOutputRevolution = 262144;

    // 将输出端位置计数转换为弧度。
    double positionCountsToRadians(
        std::int32_t position_counts,
        std::uint32_t counts_per_output_revolution);

    // 将输出端弧度转换为位置计数。
    std::int32_t radiansToPositionCounts(
        double radians,
        std::uint32_t counts_per_output_revolution);
}
