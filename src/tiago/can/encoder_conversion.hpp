#pragma once

#include "tiago/can/can_config.hpp"

#include <cstdint>

namespace robot::tiago
{
    // 将旋转编码器计数转换为关节角度，结果单位为弧度。
    double countsToRadians(std::int32_t counts, const RotaryEncoderConfig &config);

    // 将关节角度转换为旋转编码器计数，输入角度单位为弧度。
    std::int32_t radiansToCounts(double radians, const RotaryEncoderConfig &config);

    // 将直线编码器计数转换为关节位移，结果单位为米。
    double countsToMeters(std::int32_t counts, const LinearEncoderConfig &config);

    // 将关节位移转换为直线编码器计数，输入位移单位为米。
    std::int32_t metersToCounts(double meters, const LinearEncoderConfig &config);

    // 将旋转关节的速度上限从弧度/秒转换为编码器计数/秒。
    std::uint16_t radiansPerSecondToCountsPerSecond(double radians_per_second, const RotaryEncoderConfig &config);

    // 将直线关节的速度上限从米/秒转换为编码器计数/秒。
    std::uint16_t metersPerSecondToCountsPerSecond(double meters_per_second, const LinearEncoderConfig &config);
}
