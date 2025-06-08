#pragma once

#include "pros/distance.hpp"
#include "units/length.h"
#include <cstdint>

using namespace units;

namespace aekulib
{
    class Distance
    {
      public:
        explicit Distance(const std::uint8_t port);

        inches<> get() const;

        double getConfidence() const;

      private:
        std::shared_ptr<pros::Distance> distance_;
    };
}