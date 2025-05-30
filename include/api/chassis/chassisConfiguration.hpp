#pragma once

#include "api/units.h"
#include "pros/abstract_motor.hpp"

using namespace units::literals;

using namespace units::length;

namespace aekulib 
{
    class ChassisConfiguration
    {
    public:
        ChassisConfiguration(const inch_t iwheelDiameter,
                             const inch_t itrackWidth,
                             const pros::MotorGears &igearset,
                             const double igearRatio);
    
        inline inch_t getWheelDiameter() const { return wheelDiameter; }
        inline inch_t getTrackWidth() const { return trackWidth; }
        inline pros::MotorGears getGearset() const { return gearset; }
        inline double getGearRatio() const { return gearRatio; }
    
    private:
        inch_t wheelDiameter, trackWidth;
        pros::MotorGears gearset;
        double gearRatio;
    };
}