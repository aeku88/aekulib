#pragma once

#include "odometry/odometry.hpp"
#include "units/length.h"
#include "units/angle.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"
#include <cstdint>
#include <memory>
#include <cmath>
using namespace units;
using namespace units::literals;
namespace aekulib
{
    struct RamseteGains
    {
      public:
        RamseteGains(const double ib, const double izeta) : b(ib), zeta(izeta) {}
        double getB() const { return b; }
        double getZeta() const { return zeta; }

      private:
        double b, zeta;
    };

    class RamseteController
    {
      public:
        explicit RamseteController(const std::shared_ptr<RamseteGains> &iramseteGains,
                                   const std::shared_ptr<Odometry> &iodometry);

        void setTargetValues(inches<> x, inches<> y, degrees<> theta);
        void step();
        std::pair<inches_per_second<>, degrees_per_second<>> getVelocities();

      private:
        std::shared_ptr<Odometry> odometry = nullptr;
        std::shared_ptr<RamseteGains> ramsetegains;
        inches<> x_d = 0_in;
        inches<> y_d = 0_in;
        degrees<> theta_d = radians(0);
        inches_per_second<> v = inches_per_second(0);
        degrees_per_second<> omega = degrees_per_second(0);
    };
}