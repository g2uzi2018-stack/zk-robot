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

    // 将 CSP 反馈的电机轴速度原始值（0.01 Hz）转换为输出端 rad/s。
    double speedRawToOutputRadiansPerSecond(
        std::int16_t speed_raw,
        double gear_ratio);
}
