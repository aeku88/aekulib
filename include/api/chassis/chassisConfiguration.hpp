#pragma once

#include "pros/abstract_motor.hpp"

#include "units/length.h"
#include "units/mass.h"

using namespace units;

namespace aekulib
{
    class ChassisConfiguration
    {
      public:
        ChassisConfiguration(const inches<> iwheelDiameter, const inches<> itrackWidth,
                             const pros::MotorGears &igearset, const double igearRatio,
                             const kilograms<> imass)
            : wheelDiameter(iwheelDiameter), trackWidth(itrackWidth), gearset(igearset),
              gearRatio(igearRatio), mass(imass)
        {}

        inline inches<> getWheelDiameter() const { return wheelDiameter; }
        inline inches<> getTrackWidth() const { return trackWidth; }
        inline pros::MotorGears getGearset() const { return gearset; }
        inline double getGearRatio() const { return gearRatio; }
        inline kilograms<> getMass() const { return mass; }

      private:
        inches<> wheelDiameter, trackWidth;
        pros::MotorGears gearset;
        kilograms<> mass;
        double gearRatio;
    };
}