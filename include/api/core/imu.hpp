#pragma once

#include "pros/imu.hpp"
#include "api/units.h"
#include "Eigen/Geometry"

using namespace units::literals;

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
        units::angle::degree_t getHeading() const;
        void setHeading(units::angle::degree_t target);

        units::angle::degree_t getYaw() const;
        void setYaw(units::angle::degree_t target);

        units::angle::degree_t getPitch() const;
        void setPitch(units::angle::degree_t target);

        units::angle::degree_t getRoll() const;
        void setRoll(units::angle::degree_t target);

        // Euler angles structure
        struct Euler
        {
            units::angle::degree_t roll;
            units::angle::degree_t pitch;
            units::angle::degree_t yaw;
        };

        Euler getEuler() const;
        void setEuler(const Euler &target);

        // Total rotation around the Z axis since initialization (degrees)
        units::angle::degree_t getRotation() const;

        // Raw gyroscope rates (degrees per second)
        struct Gyro
        {
            units::angular_velocity::degrees_per_second_t x;
            units::angular_velocity::degrees_per_second_t y;
            units::angular_velocity::degrees_per_second_t z;
        };

        Gyro getGyroRate() const;

        // Linear acceleration (m/s^2)
        struct Accel
        {
            units::acceleration::meters_per_second_squared_t x;
            units::acceleration::meters_per_second_squared_t y;
            units::acceleration::meters_per_second_squared_t z;
        };

        Accel getAccel() const;

        // Orientation quaternion
        Eigen::Quaterniond getQuaternion() const;
        
      private:
        pros::Imu imu_;
    };
}