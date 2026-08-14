#include "tiago/torso/torso.hpp"

#include <stdexcept>

namespace robot::tiago
{
    const JointConfig &Torso::validateConfig(const CanBusConfig &config)
    {
        if (config.joints.size() != 1)
        {
            throw std::invalid_argument("Torso config must contain exactly 1 joint");
        }

        const auto &joint_config = config.joints[0];

        if (joint_config.motor.unit != JointUnit::Meter)
        {
            throw std::invalid_argument("Torso joint must use meter units");
        }

        return joint_config;
    }

    Torso::Torso(const CanBusConfig &config)
        : bus_(config.interface_name),
          joint_(validateConfig(config), bus_)
    {
    }

    void Torso::enable()
    {
        joint_.enable();
    }

    void Torso::disable()
    {
        joint_.disable();
    }

    void Torso::clearFault()
    {
        joint_.clearFault();
    }

    void Torso::stop()
    {
        joint_.stop();
    }

    void Torso::commandPosition(double position, double velocity_limit)
    {
        joint_.commandPosition(position, velocity_limit);
    }

    std::optional<double> Torso::readPosition()
    {
        return joint_.readPosition();
    }

    Joint &Torso::joint()
    {
        return joint_;
    }

    const Joint &Torso::joint() const
    {
        return joint_;
    }
}
