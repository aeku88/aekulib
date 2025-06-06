#include "api/core/distance.hpp"

namespace aekulib
{
    Distance::Distance(const std::uint8_t port)
        : distance_(std::make_shared<pros::Distance>(port))
    {}

    inches<> Distance::get() const
    {
        return millimeters(distance_->get());
    }

    double Distance::getConfidence() const
    {
        return distance_->get_confidence();
    }
}