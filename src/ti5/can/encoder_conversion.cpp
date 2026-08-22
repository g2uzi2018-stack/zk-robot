#include "ti5/can/encoder_conversion.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace
{
    // 使用显式常量，避免依赖平台对 M_PI 的扩展定义。
    constexpr double kTwoPi = 6.283185307179586476925286766559;

    void validateCountsPerOutputRevolution(std::uint32_t counts_per_output_revolution)
    {
        if (counts_per_output_revolution == 0)
        {
            throw std::invalid_argument("counts_per_output_revolution must be positive");
        }
    }
}

namespace robot::ti5
{
    // 位置计数是输出端计数，不再除以 gear_ratio。
    double positionCountsToRadians(
        std::int32_t position_counts,
        std::uint32_t counts_per_output_revolution)
    {
        validateCountsPerOutputRevolution(counts_per_output_revolution);

        return static_cast<double>(position_counts) * kTwoPi /
               static_cast<double>(counts_per_output_revolution);
    }

    // 协议文档规定原始位置由浮点结果截断得到，因此这里向零截断。
    std::int32_t radiansToPositionCounts(
        double radians,
        std::uint32_t counts_per_output_revolution)
    {
        validateCountsPerOutputRevolution(counts_per_output_revolution);

        if (!std::isfinite(radians))
        {
            throw std::invalid_argument("radians must be finite");
        }

        const double raw_counts = radians *
                                   static_cast<double>(counts_per_output_revolution) /
                                   kTwoPi;
        const double truncated_counts = std::trunc(raw_counts);

        if (truncated_counts < static_cast<double>(std::numeric_limits<std::int32_t>::min()) ||
            truncated_counts > static_cast<double>(std::numeric_limits<std::int32_t>::max()))
        {
            throw std::out_of_range("position counts exceed int32 protocol range");
        }

        return static_cast<std::int32_t>(truncated_counts);
    }

    double speedRawToOutputRadiansPerSecond(
        std::int16_t speed_raw,
        double gear_ratio)
    {
        if (!(gear_ratio > 0.0) || !std::isfinite(gear_ratio))
        {
            throw std::invalid_argument("gear_ratio must be positive and finite");
        }

        constexpr double kTwoPi = 6.283185307179586476925286766559;
        return static_cast<double>(speed_raw) * kTwoPi /
               (100.0 * gear_ratio);
    }
}
