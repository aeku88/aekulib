#pragma once

#include "api/chassis/chassisConfiguration.hpp"

#include "Eigen/Eigen"
#include "units/angular_velocity.h"
#include "units/velocity.h"

using namespace units;

namespace aekulib
{
    class ChassisKinematics
    {
      public:
        ChassisKinematics(const std::shared_ptr<ChassisConfiguration> &iconfig);

        std::pair<inches_per_second<>, radians_per_second<>>
        forward(const Eigen::Vector2<revolutions_per_minute<>> iwheelVelocities);

        Eigen::Vector2<revolutions_per_minute<>>
        inverse(const std::pair<inches_per_second<>, radians_per_second<>> ichassisVelocities);

      private:
        std::shared_ptr<ChassisConfiguration> m_config;
    };
}