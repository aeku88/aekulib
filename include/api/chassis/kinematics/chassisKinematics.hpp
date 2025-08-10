#pragma once

#include "api/chassis/chassisConfiguration.hpp"

#include "Eigen/Eigen"
#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/math/geometry/twist2D.hpp"
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

        Twist2D toTwist2D(Eigen::Vector2<inches<>> distances) const
        {
            return {(distances[0] + distances[1]) / 2, 0_m,
                    (distances[1] - distances[0]) / m_config->getTrackWidth() * 1_rad};
        }

        Twist2D toTwist2D(Eigen::Vector2<inches<>> currentPositions, Eigen::Vector2<inches<>> lastPositions)
        {
            return toTwist2D(currentPositions - lastPositions);
        }

      private:
        std::shared_ptr<ChassisConfiguration> m_config;
    };
}