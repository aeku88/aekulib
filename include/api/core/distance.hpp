#pragma once

#include "pros/distance.hpp"
#include "api/units.h"
#include <cstdint>

namespace aekulib
{
    class Distance
    {
      public:
        explicit Distance(const std::uint8_t port);

        units::length::millimeter_t get() const;

        double getConfidence() const;

      private:
        std::shared_ptr<pros::Distance> distance_;
    };
}