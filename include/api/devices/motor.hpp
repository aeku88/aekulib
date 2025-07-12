#pragma once

#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"

#include "units/angular_velocity.h"
#include "units/current.h"
#include "units/power.h"
#include "units/temperature.h"
#include "units/torque.h"
#include "units/voltage.h"
#include <cstdint>

using namespace units;

namespace aekulib
{
    class Motor
    {
      public:
        /**
         * @param port PROS motor port number (negative for reversed direction)
         */
        explicit Motor(const std::int8_t port,
                       const pros::v5::MotorGears gearset = pros::v5::MotorGears::invalid);

        void move(const volts<> voltage) const;

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
        degrees<> getPosition() const;

        /**
         * Get target velocity (RPM) setpoint.
         */
        revolutions_per_minute<> getTargetVelocity() const;

        /**
         * Get actual motor velocity (RPM).
         */
        revolutions_per_minute<> getActualVelocity() const;

        /**
         * Get motor current draw (amps).
         */
        amperes<> getCurrentDraw() const;

        std::int32_t getDirection() const;

        /**
         * Get motor temperature (Celsius).
         */
        celsius<> getTemperature() const;

        /**
         * Get motor efficiency (0.0-1.0).
         */
        double getEfficiency() const;

        /**
         * Get motor torque.
         */
        newton_meters<> getTorque() const;

        /**
         * Get motor power.
         */
        watts<> getPower() const;

        /**
         * Get motor voltage (millivolts).
         */
        volts<> getVoltage() const;

      private:
        std::unique_ptr<pros::Motor> m_motor;
    };
}