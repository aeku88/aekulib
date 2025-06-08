#include "api/devices/motor.hpp"

namespace aekulib
{
    Motor::Motor(const std::int8_t port, const pros::v5::MotorGears gearset)
        : m_motor(std::make_shared<pros::Motor>(port, gearset))
    {
        m_motor->set_encoder_units(pros::MotorEncoderUnits::degrees);
    }

    void Motor::move(const volts<> voltage) const { m_motor->move_voltage(voltage.to<int32_t>()); }

    void Motor::brake() { m_motor->brake(); }

    void Motor::setBrakeMode(const pros::MotorBrake mode) const { m_motor->set_brake_mode(mode); }

    void Motor::resetPosition() const { m_motor->tare_position(); }

    degrees<> Motor::getPosition() const { return degrees(m_motor->get_position()); }

    revolutions_per_minute<> Motor::getTargetVelocity() const
    {
        return revolutions_per_minute(m_motor->get_target_velocity());
    }

    revolutions_per_minute<> Motor::getActualVelocity() const
    {
        return revolutions_per_minute(m_motor->get_actual_velocity());
    }

    amperes<> Motor::getCurrentDraw() const { return milliamperes(m_motor->get_current_draw()); }

    std::int32_t Motor::getDirection() const { return m_motor->get_direction(); }

    celsius<> Motor::getTemperature() const { return celsius(m_motor->get_temperature()); }

    double Motor::getEfficiency() const { return m_motor->get_efficiency(); }

    newton_meters<> Motor::getTorque() const { return newton_meters(m_motor->get_torque()); }

    watts<> Motor::getPower() const { return watts(m_motor->get_power()); }

    volts<> Motor::getVoltage() const { return millivolts(m_motor->get_voltage()); }
}