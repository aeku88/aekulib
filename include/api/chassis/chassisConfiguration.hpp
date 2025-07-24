#pragma once

#include "api/devices/rotationSensor.hpp"
#include "api/devices/imu.hpp"
#include "pros/abstract_motor.hpp"

#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/mass.h"
#include "units/velocity.h"
#include <cstddef>

using namespace units;
using namespace units::literals;

namespace aekulib
{
    struct ChassisSensors
    {
      public:
        ChassisSensors(const std::shared_ptr<RotationSensor> ileft,
                       const std::shared_ptr<RotationSensor> iright,
                       const std::shared_ptr<RotationSensor> imiddle, const std::shared_ptr<IMU> iimu)
            : left(ileft), right(iright), middle(imiddle), imu(iimu)
        {}

        Eigen::Vector2<inches<>> getEncoderVals() const;
        degrees<> getOrientation() const;

      private:
        std::shared_ptr<RotationSensor> left = nullptr, right = nullptr, middle = nullptr;
        std::shared_ptr<IMU> imu = nullptr;
    };

    class ChassisConfiguration
    {
      public:
        ChassisConfiguration(const inches<> iwheelDiameter, const inches<> itrackWidth,
                             const pros::MotorGears &igearset, const double igearRatio,
                             const kilograms<> imass)
            : wheelDiameter(iwheelDiameter), trackWidth(itrackWidth), gearset(igearset),
              gearRatio(igearRatio), mass(imass)
        {
            switch(gearset)
            {
            case pros::MotorGears::ratio_36_to_1: motorVelocity = 100_rpm;
            case pros::MotorGears::ratio_18_to_1: motorVelocity = 200_rpm;
            case pros::MotorGears::ratio_6_to_1: motorVelocity = 600_rpm;
            case pros::MotorGears::invalid: break;
            }
        }

        inline inches<> getWheelDiameter() const { return wheelDiameter; }
        inline inches<> getTrackWidth() const { return trackWidth; }
        inline pros::MotorGears getGearset() const { return gearset; }
        inline double getGearRatio() const { return gearRatio; }
        inline std::pair<inches_per_second<>, radians_per_second<>> getMaxVelocities() const
        {
            return {gearRatio * motorVelocity * getWheelDiameter() * M_PI / 1_tr,
                    (1_rad * gearRatio * motorVelocity * getWheelDiameter() * M_PI / 1_tr) / trackWidth};
        }
        inline kilograms<> getMass() const { return mass; }

      private:
        inches<> wheelDiameter, trackWidth;
        revolutions_per_minute<> motorVelocity;
        pros::MotorGears gearset;
        kilograms<> mass;
        double gearRatio;
    };
}