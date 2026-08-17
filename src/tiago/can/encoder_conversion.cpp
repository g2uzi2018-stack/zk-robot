#include "tiago/can/encoder_conversion.hpp"

#include <limits>
#include <cmath>
#include <stdexcept>

namespace robot::tiago
{
    // 计数 -> 电机转数 -> 关节转数 -> 关节弧度，并应用方向和零点偏移。
    double countsToRadians(std::int32_t counts, const RotaryEncoderConfig &config)
    {
        // 编码器分辨率为零时无法进行换算。
        if (config.counts_per_motor_revolution == 0)
        {
            throw std::invalid_argument("counts_per_motor_revolution cannot be zero");
        }

        const double motor_revolutions = static_cast<double>(counts) / static_cast<double>(config.counts_per_motor_revolution);
        const double joint_revolutions = motor_revolutions / config.gear_ratio;
        double radians = joint_revolutions * 2.0 * M_PI;
        radians *= config.direction;
        radians += config.zero_offset;
        return radians;
    }

    // 关节弧度 -> 关节转数 -> 电机转数 -> 编码器计数，并应用方向和零点偏移。
    std::int32_t radiansToCounts(double radians, const RotaryEncoderConfig &config)
    {
        // 减速比必须为正数，否则换算结果没有物理意义。
        if (config.gear_ratio <= 0.0)
        {
            throw std::invalid_argument("gear_ratio must be positive");
        }

        const double joint_revolutions = (radians - config.zero_offset) / (2.0 * M_PI);
        const double motor_revolutions = joint_revolutions * config.gear_ratio;
        double counts = motor_revolutions * static_cast<double>(config.counts_per_motor_revolution);
        counts *= config.direction;
        // 编码器计数必须是整数，因此这里采用四舍五入。
        return static_cast<std::int32_t>(std::round(counts));
    }

    // 计数 -> 位移，并应用方向和零点偏移。
    double countsToMeters(std::int32_t counts, const LinearEncoderConfig &config)
    {
        // 每米计数必须为正数，否则无法进行有效换算。
        if (config.counts_per_meter <= 0.0)
        {
            throw std::invalid_argument("counts_per_meter must be positive");
        }

        double meters = static_cast<double>(counts) / config.counts_per_meter;
        meters *= config.direction;
        meters += config.zero_offset;
        return meters;
    }

    // 位移 -> 编码器计数，并应用方向和零点偏移。
    std::int32_t metersToCounts(double meters, const LinearEncoderConfig &config)
    {
        // 每米计数必须为正数，否则无法进行有效换算。
        if (config.counts_per_meter <= 0.0)
        {
            throw std::invalid_argument("counts_per_meter must be positive");
        }

        double counts = (meters - config.zero_offset) * config.counts_per_meter;
        counts *= config.direction;
        // 编码器计数必须是整数，因此这里采用四舍五入。
        return static_cast<std::int32_t>(std::round(counts));
    }

    // 将弧度/秒转换为电机编码器计数/秒，不应用零点偏移。
    std::uint16_t radiansPerSecondToCountsPerSecond(double radians_per_second, const RotaryEncoderConfig &config)
    {
        // 编码器分辨率和减速比必须有效。
        if (config.counts_per_motor_revolution == 0)
        {
            throw std::invalid_argument("counts_per_motor_revolution cannot be zero");
        }
        if (config.gear_ratio <= 0.0)
        {
            throw std::invalid_argument("gear_ratio must be positive");
        }
        // 速度上限只接受非负值。
        if (radians_per_second < 0.0)
        {
            throw std::invalid_argument("radians_per_second must not be negative");
        }

        const double joint_revolutions_per_second = radians_per_second / (2.0 * M_PI);
        const double motor_revolutions_per_second = joint_revolutions_per_second * config.gear_ratio;
        const double counts_per_second = motor_revolutions_per_second * static_cast<double>(config.counts_per_motor_revolution);
        // CAN 协议中的速度字段使用无符号 16 位整数。
        return static_cast<std::uint16_t>(std::round(counts_per_second));
    }

    // 将米/秒转换为直线编码器计数/秒，不应用零点偏移。
    std::uint16_t metersPerSecondToCountsPerSecond(double meters_per_second, const LinearEncoderConfig &config)
    {
        // 每米计数必须为正数。
        if (config.counts_per_meter <= 0.0)
        {
            throw std::invalid_argument("counts_per_meter must be positive");
        }
        // 速度上限只接受非负值。
        if (meters_per_second < 0.0)
        {
            throw std::invalid_argument("meters_per_second must not be negative");
        }

        const double counts_per_second = meters_per_second * config.counts_per_meter;
        // CAN 协议中的速度字段使用无符号 16 位整数。
        return static_cast<std::uint16_t>(std::round(counts_per_second));
    }

    // 将旋转关节速度转换为有符号编码器计数速度。
    std::int32_t radiansPerSecondToSignedCountsPerSecond(
        double radians_per_second,
        const RotaryEncoderConfig &config)
    {
        if (!std::isfinite(radians_per_second))
        {
            throw std::invalid_argument(
                "radians_per_second must be finite");
        }

        if (config.counts_per_motor_revolution == 0)
        {
            throw std::invalid_argument(
                "counts_per_motor_revolution cannot be zero");
        }

        if (config.gear_ratio <= 0.0)
        {
            throw std::invalid_argument(
                "gear_ratio must be positive");
        }

        const double joint_revolutions_per_second =
            radians_per_second / (2.0 * M_PI);

        const double motor_revolutions_per_second =
            joint_revolutions_per_second *
            config.gear_ratio;
        double counts_per_second =
            motor_revolutions_per_second *
            static_cast<double>(
                config.counts_per_motor_revolution);
        counts_per_second *= config.direction;
        const double rounded =
            std::round(counts_per_second);

        if (rounded <
                static_cast<double>(
                    std::numeric_limits<std::int32_t>::min()) ||
            rounded >
                static_cast<double>(
                    std::numeric_limits<std::int32_t>::max()))
        {
            throw std::out_of_range(
                "Velocity exceeds int32 protocol range");
        }

        return static_cast<std::int32_t>(rounded);
    }

    // 将有符号编码器计数速度转换为旋转关节速度。
    double countsPerSecondToRadiansPerSecond(std::int32_t counts_per_second, const RotaryEncoderConfig &config)
    {
        if (config.counts_per_motor_revolution == 0)
        {
            throw std::invalid_argument("counts_per_motor_revolution cannot be zero");
        }

        if (config.gear_ratio <= 0.0)
        {
            throw std::invalid_argument("gear_ratio must be positive");
        }

        const double motor_revolutions_per_second = static_cast<double>(counts_per_second) / static_cast<double>(config.counts_per_motor_revolution);
        const double joint_revolutions_per_second = motor_revolutions_per_second / config.gear_ratio;
        double radians_per_second = joint_revolutions_per_second * 2.0 * M_PI;
        radians_per_second *= config.direction;
        return radians_per_second;
    }
}
