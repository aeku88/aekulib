#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "units.h"
#include "units/velocity.h"
#include <cmath>

namespace aekulib
{
    ChassisKinematics::ChassisKinematics(
      const std::shared_ptr<ChassisConfiguration> &iconfig)
        : config(iconfig)
    {}

    std::pair<inches_per_second<>, radians_per_second<>> ChassisKinematics::forward(
        const Eigen::Vector2<revolutions_per_minute<>> iwheelVelocities)
    {
        revolutions_per_minute leftVel = iwheelVelocities[0],
                             rightVel = iwheelVelocities[1];

        inches_per_second linVel
          = (config->getWheelDiameter() * (leftVel + rightVel)) / (4 * rad);
        radians_per_second angVel
          = (config->getWheelDiameter() * (rightVel - leftVel))
            / (2 * config->getTrackWidth());

        return {linVel, angVel};
    }

    Eigen::Vector2<revolutions_per_minute<>> ChassisKinematics::inverse(
      const std::pair<inches_per_second<>, radians_per_second<>>
        ichassisVelocities)
    {
        inches_per_second<> linVel = ichassisVelocities.first;
        radians_per_second<> angVel = ichassisVelocities.second;

        radians_per_second<> leftVel
          = (linVel * rad + (config->getTrackWidth() / 2) * angVel)
            / (config->getWheelDiameter()
               / 2); // 1_rad multiplicant for units match; angle has dimension
                     // in lemlib/units implementation
        radians_per_second<> rightVel
          = (linVel * rad - (config->getTrackWidth() / 2) * angVel)
            / (config->getWheelDiameter() / 2);

        return {leftVel, rightVel};
    }
}