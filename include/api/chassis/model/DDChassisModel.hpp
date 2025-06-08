#pragma once

#include "api/devices/motorGroup.hpp"

#include "Eigen/Eigen"
#include "units/angular_velocity.h"

namespace aekulib
{
    class DifferentialDriveChassisModel
    {
      public:
        virtual void drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities) = 0;

        virtual void step() = 0;
    };
}