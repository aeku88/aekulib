#pragma once

#include "api/chassis/chassisConfiguration.hpp"
#include "api/devices/motorGroup.hpp"

#include "Eigen/Eigen"
#include "units/angular_velocity.h"
#include "units/mass.h"

namespace aekulib
{
    class DifferentialDriveChassisModel
    {
      public:
        // DifferentialDriveChassisModel(const ChassisConfiguration &iconfig) {}

        virtual void drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities) = 0;

        virtual void step() = 0;
    };
}