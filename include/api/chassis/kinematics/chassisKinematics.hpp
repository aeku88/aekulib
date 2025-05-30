#pragma once

#include "Eigen/Eigen"
#include "api/units.h"
#include "api/chassis/chassisConfiguration.hpp"

using namespace units::literals;

using namespace units::velocity;
using namespace units::angular_velocity;

namespace aekulib
{
    class ChassisKinematics
    {
      public:
        ChassisKinematics(const std::shared_ptr<ChassisConfiguration> &iconfig);

        std::pair<feet_per_second_t, radians_per_second_t> forward(
          const Eigen::Vector2<revolutions_per_minute_t> iwheelVelocities);

        Eigen::Vector2<revolutions_per_minute_t>
        inverse(const std::pair<feet_per_second_t, radians_per_second_t>
                  ichassisVelocities);
    private:
        std::shared_ptr<ChassisConfiguration> config; 
    };
}