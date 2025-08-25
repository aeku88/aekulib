#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "units.h"
#include "units/core.h"
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

        auto leftVel = (linVel.value() + m_config->getTrackWidth().value() / 2.0 * angVel.value())
                       * ips; // rad unit constant for units match; angle has
                              // dimension in nholthaus/units implementation
        auto rightVel = (linVel.value() - m_config->getTrackWidth().value() / 2.0 * angVel.value()) * ips;
        std::cout << linVel << ", " << angVel << '\n';
        auto max_wheel_vel = std::max(abs(leftVel), abs(rightVel));

        if(max_wheel_vel > m_config->getMaxVelocities().first)
        {
            // normalize the wheel velocities
            leftVel = leftVel / max_wheel_vel * m_config->getMaxVelocities().first;
            rightVel = rightVel / max_wheel_vel * m_config->getMaxVelocities().first;
        }

        return {leftVel * (360_deg / (m_config->getWheelDiameter() * pi)) * m_config->getGearRatio(),
                rightVel * (360_deg / (m_config->getWheelDiameter() * pi)) * m_config->getGearRatio()};
    }

    Eigen::Vector2<revolutions_per_minute<>>
    ChassisKinematics::inverse(const inches_per_second<> linVel, double curvature)
    {
        if(abs(linVel) < 1e-4_ips)
            return {0_rpm, 0_rpm};
        else if(fabs(curvature) < 1e-4)
            return inverse({linVel, 0_rps});

        return inverse({linVel, linVel * curvature * rad / in});
    }
}