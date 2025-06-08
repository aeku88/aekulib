#pragma once

#include "api/odometry/odometry.hpp"

#include <cstdint>
#include <memory>

namespace aekulib
{
    struct RamseteGains
    {
      public:
        RamseteGains(const double ib, const double izeta) : b(ib), zeta(izeta) {}

      private:
        double b, zeta;
    };

    class RamseteController
    {
      public:
        explicit RamseteController(const std::shared_ptr<RamseteGains> &iramseteGains,
                                   const std::shared_ptr<Odometry> &iodometry);

        void step();

      private:
        std::shared_ptr<Odometry> odometry = nullptr;
    };
}