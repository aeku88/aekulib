#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "units.h"
#include "units/velocity.h"
#include <cmath>

namespace aekulib
{
    ChassisKinematics::ChassisKinematics(const std::shared_ptr<ChassisConfiguration> &config)
        : m_config(config)
    {}

    std::pair<inches_per_second<>, radians_per_second<>>
    ChassisKinematics::forward(const Eigen::Vector2<revolutions_per_minute<>> iwheelVelocities)
    {
        revolutions_per_minute leftVel = iwheelVelocities[0], rightVel = iwheelVelocities[1];

        inches_per_second linVel = (m_config->getWheelDiameter() * (leftVel + rightVel)) / (4 * rad);
        radians_per_second angVel
          = (m_config->getWheelDiameter() * (rightVel - leftVel)) / (2 * m_config->getTrackWidth());

        return {linVel, angVel};
    }

    Eigen::Vector2<revolutions_per_minute<>>
    ChassisKinematics::inverse(const std::pair<inches_per_second<>, radians_per_second<>> ichassisVelocities)
    {
        inches_per_second<> linVel = ichassisVelocities.first;
        radians_per_second<> angVel = ichassisVelocities.second;

        radians_per_second<> leftVel
          = (linVel * rad + (m_config->getTrackWidth() / 2) * angVel)
            / (m_config->getWheelDiameter() / 2); // rad unit constant for units match; angle has
                                                  // dimension in nholthaus/units implementation
        radians_per_second<> rightVel
          = (linVel * rad - (m_config->getTrackWidth() / 2) * angVel) / (m_config->getWheelDiameter() / 2);

        return {leftVel, rightVel};
    }
}