#include "api/core/motorGroup.hpp"
#include "api/units.h"
#include <cstdint>

namespace aekulib
{
    MotorGroup::MotorGroup(const std::initializer_list<std::int8_t> ports, const pros::v5::MotorGears gearset)
        : motor_(std::make_shared<pros::MotorGroup>(ports, gearset))
    {
        motor_->set_encoder_units(pros::MotorEncoderUnits::degrees);
    }

    void MotorGroup::move(const units::voltage::millivolt_t voltage) const
    {
        motor_->move_voltage(voltage.to<int32_t>());
    }

    void MotorGroup::move(
      const units::angular_velocity::revolutions_per_minute_t rpm) const
    {
        motor_->move_velocity(rpm.to<int32_t>());
    }

    void MotorGroup::brake() { motor_->brake(); }

    void MotorGroup::setBrakeMode(const pros::MotorBrake mode) const
    {
        motor_->set_brake_mode(mode);
    }

    void MotorGroup::resetPosition() const { motor_->tare_position(); }

    units::angle::degree_t MotorGroup::getPosition() const
    {
        return units::angle::degree_t(motor_->get_position());
    }

    units::angular_velocity::revolutions_per_minute_t
    MotorGroup::getTargetVelocity() const
    {
        return units::angular_velocity::revolutions_per_minute_t(
          motor_->get_target_velocity());
    }

    units::angular_velocity::revolutions_per_minute_t
    MotorGroup::getActualVelocity() const
    {
        return units::angular_velocity::revolutions_per_minute_t(
          motor_->get_actual_velocity());
    }

    units::current::ampere_t MotorGroup::getCurrentDraw() const
    {
        return units::current::ampere_t(motor_->get_current_draw());
    }

    std::int32_t MotorGroup::getDirection() const
    {
        return motor_->get_direction();
    }

    units::temperature::celsius_t MotorGroup::getTemperature() const
    {
        return units::temperature::celsius_t(motor_->get_temperature());
    }

    double MotorGroup::getEfficiency() const { return motor_->get_efficiency(); }

    units::torque::newton_meter_t MotorGroup::getTorque() const
    {
        return units::torque::newton_meter_t(motor_->get_torque());
    }

    units::power::milliwatt_t MotorGroup::getPower() const
    {
        return units::power::milliwatt_t(motor_->get_power());
    }
}