#include "tiago/base/base.hpp"

#include <cmath>
#include <stdexcept>

namespace robot::tiago
{
    Base::Base(const BaseConfig &config)
        : bus_(config.interface_name),
          right_motor_(config.right_motor, bus_),
          left_motor_(config.left_motor, bus_),
          wheel_radius_(config.wheel_radius),
          wheel_separation_(config.wheel_separation),
          max_linear_velocity_(config.max_linear_velocity),
          max_angular_velocity_(config.max_angular_velocity)
    {
    }

    void Base::enable()
    {
        right_motor_.enable();
        left_motor_.enable();
    }

    void Base::disable()
    {
        right_motor_.disable();
        left_motor_.disable();
    }

    void Base::clearFault()
    {
        right_motor_.clearFault();
        left_motor_.clearFault();
    }

    void Base::stop()
    {
        right_motor_.stop();
        left_motor_.stop();
    }

    void Base::validateVelocityCommand(double linear_velocity, double angular_velocity) const
    {
        if (!std::isfinite(linear_velocity) || !std::isfinite(angular_velocity))
        {
            throw std::invalid_argument("Base velocity command must be finite");
        }

        if (std::abs(linear_velocity) > max_linear_velocity_)
        {
            throw std::out_of_range("Base linear velocity exceeds limit");
        }

        if (std::abs(angular_velocity) > max_angular_velocity_)
        {
            throw std::out_of_range("Base angular velocity exceeds limit");
        }
    }

    void Base::commandVelocity(double linear_velocity, double angular_velocity)
    {
        validateVelocityCommand(linear_velocity, angular_velocity);

        // 差速底盘运动学：v_right = v + wL/2，v_left = v - wL/2。
        // 这里的 v 是车体线速度，w 是车体角速度，L 是轮距。
        const double right_velocity =
            (linear_velocity + angular_velocity * wheel_separation_ / 2.0) /
            wheel_radius_;
        const double left_velocity =
            (linear_velocity - angular_velocity * wheel_separation_ / 2.0) /
            wheel_radius_;

        // 电机接口使用车轮角速度，因此需要除以车轮半径后再发送。
        right_motor_.commandVelocity(right_velocity);
        left_motor_.commandVelocity(left_velocity);
    }
}
