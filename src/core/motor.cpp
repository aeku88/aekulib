#include "api/core/motor.hpp"
#include "api/units.h"
#include <cstdint>

namespace aekulib
{
    Motor::Motor(const std::int8_t port, const pros::v5::MotorGears gearset)
        : motor_(std::make_shared<pros::Motor>(port, gearset))
    {
        motor_->set_encoder_units(pros::MotorEncoderUnits::degrees);
    }

    void Motor::move(const units::voltage::millivolt_t voltage) const
    {
        motor_->move_voltage(voltage.to<int32_t>());
    }

    void Motor::move(
      const units::angular_velocity::revolutions_per_minute_t rpm) const
    {
        motor_->move_velocity(rpm.to<int32_t>());
    }

    void Motor::brake() { motor_->brake(); }

    void Motor::setBrakeMode(const pros::MotorBrake mode) const
    {
        motor_->set_brake_mode(mode);
    }

    void Motor::resetPosition() const { motor_->tare_position(); }

    units::angle::degree_t Motor::getPosition() const
    {
        return units::angle::degree_t(motor_->get_position());
    }

    units::angular_velocity::revolutions_per_minute_t
    Motor::getTargetVelocity() const
    {
        return units::angular_velocity::revolutions_per_minute_t(
          motor_->get_target_velocity());
    }

    units::angular_velocity::revolutions_per_minute_t
    Motor::getActualVelocity() const
    {
        return units::angular_velocity::revolutions_per_minute_t(
          motor_->get_actual_velocity());
    }

    units::current::ampere_t Motor::getCurrentDraw() const
    {
        return units::current::ampere_t(motor_->get_current_draw());
    }

    std::int32_t Motor::getDirection() const
    {
        return motor_->get_direction();
    }

    units::temperature::celsius_t Motor::getTemperature() const
    {
        return units::temperature::celsius_t(motor_->get_temperature());
    }

    double Motor::getEfficiency() const { return motor_->get_efficiency(); }

    units::torque::newton_meter_t Motor::getTorque() const
    {
        return units::torque::newton_meter_t(motor_->get_torque());
    }

    units::power::milliwatt_t Motor::getPower() const
    {
        return units::power::milliwatt_t(motor_->get_power());
    }
}