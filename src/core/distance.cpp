#include "api/core/distance.hpp"

namespace aekulib
{
    Distance::Distance(const std::uint8_t port)
        : distance_(std::make_shared<pros::Distance>(port))
    {
    }

    units::length::millimeter_t Distance::get() const
    {
        return units::length::millimeter_t(distance_->get());
    }

    double Distance::getConfidence() const
    {
        return distance_->get_distance();
    }
}