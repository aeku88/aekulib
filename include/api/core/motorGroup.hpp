#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "api/units.h"
#include <cstdint>

using namespace units::literals;

namespace aekulib
{
    class MotorGroup
    {
      public:
        /**
         * @param port PROS motor port number (negative for reversed direction)
         */
        explicit MotorGroup(const std::initializer_list<std::int8_t> ports,
                       const pros::v5::MotorGears gearset
                       = pros::v5::MotorGears::invalid);

        void move(const units::voltage::millivolt_t voltage) const;

        /**
         * Set motor velocity in RPM.
         */
        void move(const units::angular_velocity::revolutions_per_minute_t rpm) const;

        /** Stops the motor according to brake mode. */
        void brake();

        /**
         * Set the motor's brake mode.
         */
        void setBrakeMode(const pros::MotorBrake mode) const;

        /**
         * Reset the motor's encoder position to zero.
         */
        void resetPosition() const;

        /**
         * Get current motor position (degrees).
         */
        units::angle::degree_t getPosition() const;

        /**
         * Get target velocity (RPM) setpoint.
         */
        units::angular_velocity::revolutions_per_minute_t
        getTargetVelocity() const;

        /**
         * Get actual motor velocity (RPM).
         */
         units::angular_velocity::revolutions_per_minute_t
        getActualVelocity() const;

        /**
         * Get motor current draw (amps).
         */
        units::current::ampere_t getCurrentDraw() const;

        std::int32_t getDirection() const;

        /**
         * Get motor temperature (Celsius).
         */
        units::temperature::celsius_t getTemperature() const;

        /**
         * Get motor efficiency (0.0-1.0).
         */
        double getEfficiency() const;

        /**
         * Get motor torque (newton-meters).
         */
        units::torque::newton_meter_t getTorque() const;

        /**
         * Get motor power (milliwatts).
         */
        units::power::milliwatt_t getPower() const;

        /**
         * Get motor voltage (millivolts).
         */
        units::voltage::millivolt_t getVoltage() const;

      private:
        std::shared_ptr<pros::MotorGroup> motor_;
    };
}