#pragma once

#include "pros/imu.hpp"

#include "Eigen/Geometry"
#include "units.h"

using namespace units;

namespace aekulib
{
    class IMU
    {
      public:
        /**
         * @param port   PROS IMU port number (1-21)
         */
        explicit IMU(std::uint8_t port);

        // Calibration and reset
        void reset();
        bool isCalibrating() const;

        // Tare (zero) functions
        int tare();
        int tareHeading();
        int tareRotation();
        int tarePitch();
        int tareRoll();
        int tareYaw();
        int tareEuler();

        // Heading and angle getters/setters
        degrees<> getHeading() const;
        void setHeading(degrees<> target);

        degrees<> getYaw() const;
        void setYaw(degrees<> target);

        degrees<> getPitch() const;
        void setPitch(degrees<> target);

        degrees<> getRoll() const;
        void setRoll(degrees<> target);

        // Total rotation around the Z axis since initialization (degrees)
        degrees<> getRotation() const;

        Eigen::Vector3<inches_per_second_squared<>> getAccel() const;
        
      private:
        pros::Imu imu_;
    };
}