#pragma once

#include "pros/abstract_motor.hpp"
#include "units.h"

using namespace units;

namespace aekulib 
{
    class ChassisConfiguration
    {
    public:
        ChassisConfiguration(const inches<> iwheelDiameter,
                             const inches<> itrackWidth,
                             const pros::MotorGears &igearset,
                             const double igearRatio);
    
        inline inches<> getWheelDiameter() const { return wheelDiameter; }
        inline inches<> getTrackWidth() const { return trackWidth; }
        inline pros::MotorGears getGearset() const { return gearset; }
        inline double getGearRatio() const { return gearRatio; }
    
    private:
        inches<> wheelDiameter, trackWidth;
        pros::MotorGears gearset;
        double gearRatio;
    };
}