#include "api/devices/motorGroup.hpp"

namespace aekulib
{
    MotorGroup::MotorGroup(const std::initializer_list<int8_t> ports, const pros::v5::MotorGears gearset)
        : m_motorGroup(std::make_shared<pros::MotorGroup>(ports, gearset))
    {
        m_motorGroup->set_encoder_units(pros::MotorEncoderUnits::degrees);
    }

    MotorGroup::MotorGroup(const std::vector<int8_t> &ports, const pros::v5::MotorGears gearset)
        : m_motorGroup(std::make_shared<pros::MotorGroup>(ports, gearset))
    {
        m_motorGroup->set_encoder_units(pros::MotorEncoderUnits::degrees);
    }

    void MotorGroup::move(const volts<> voltage) const { m_motorGroup->move_voltage(voltage.to<int32_t>()); }

    void MotorGroup::move(const revolutions_per_minute<> omega) const
    {
        move(getCurrentDraw() * 3.84_Ohm + omega / (6.68 * rpm / V));
    }

    void MotorGroup::brake() { m_motorGroup->brake(); }

    void MotorGroup::setBrakeMode(const pros::MotorBrake mode) const { m_motorGroup->set_brake_mode(mode); }

    void MotorGroup::resetPosition() const { m_motorGroup->tare_position(); }

    degrees<> MotorGroup::getPosition() const { return degrees(m_motorGroup->get_position()); }

    revolutions_per_minute<> MotorGroup::getTargetVelocity() const
    {
        return revolutions_per_minute(m_motorGroup->get_target_velocity());
    }

    revolutions_per_minute<> MotorGroup::getActualVelocity() const
    {
        return revolutions_per_minute(m_motorGroup->get_actual_velocity());
    }

    amperes<> MotorGroup::getCurrentDraw() const { return milliamperes(m_motorGroup->get_current_draw()); }

    std::int32_t MotorGroup::getDirection() const { return m_motorGroup->get_direction(); }

    celsius<> MotorGroup::getTemperature() const { return celsius(m_motorGroup->get_temperature()); }

    double MotorGroup::getEfficiency() const { return m_motorGroup->get_efficiency(); }

    newton_meters<> MotorGroup::getTorque() const { return newton_meters(m_motorGroup->get_torque()); }

    watts<> MotorGroup::getPower() const { return watts(m_motorGroup->get_power()); }

    volts<> MotorGroup::getVoltage() const { return millivolts(m_motorGroup->get_voltage()); }
}