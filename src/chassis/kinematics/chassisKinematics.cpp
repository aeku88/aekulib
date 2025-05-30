#include "api/chassis/kinematics/chassisKinematics.hpp"

namespace aekulib 
{
    ChassisKinematics::ChassisKinematics(const std::shared_ptr<ChassisConfiguration> &iconfig)
    : config(iconfig)
    {
    }

    std::pair<feet_per_second_t, radians_per_second_t>  ChassisKinematics::forward(const Eigen::Vector2<revolutions_per_minute_t> imotorVelocities)
    {
        radians_per_second_t leftVel = imotorVelocities[0],
                             rightVel = imotorVelocities[1];

        feet_per_second_t linVel = (config->getWheelDiameter() * (leftVel + rightVel)) / 4_rad;
        radians_per_second_t angVel = (config->getWheelDiameter() * (rightVel - leftVel)) / (2 * config->getTrackWidth());

        return {linVel, angVel};
    }

    Eigen::Vector2<revolutions_per_minute_t> ChassisKinematics::inverse(const std::pair<feet_per_second_t, radians_per_second_t> ichassisVelocities)
    {
        feet_per_second_t linVel = ichassisVelocities.first;
        radians_per_second_t angVel = ichassisVelocities.second;

        revolutions_per_minute_t leftVel = (linVel * 1_rad + (config->getTrackWidth() / 2) * angVel) / (config->getWheelDiameter() / 2); // 1_rad multiplicant for units match; angle has dimension in units.h implementation
        revolutions_per_minute_t rightVel = (linVel * 1_rad - (config->getTrackWidth() / 2) * angVel) / (config->getWheelDiameter() / 2);

        return {leftVel, rightVel};
    }
}