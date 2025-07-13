#include "api/devices/rotationSensor.hpp"
#include "units/angle.h"
#include "units/angular_velocity.h"

namespace aekulib
{
    RotationSensor::RotationSensor(const std::uint8_t port)
        : m_rotation(std::make_unique<pros::Rotation>(port))
    {}

    std::int32_t RotationSensor::reset() { return m_rotation->reset(); }

    std::int32_t RotationSensor::resetPosition() { return m_rotation->reset_position(); }

    std::int32_t RotationSensor::setReversed(const bool value) { return m_rotation->set_reversed(value); }

    std::int32_t RotationSensor::setPosition(const degrees<> angle)
    {
        return m_rotation->set_position(convert<centidegrees<>>(angle).value());
    }

    degrees<> RotationSensor::getPosition() const { return centidegrees(m_rotation->get_position()); }

    degrees_per_second<> RotationSensor::getVelocity() const
    {
        return centidegrees_per_second(m_rotation->get_velocity());
    }

    degrees<> RotationSensor::getAngle() const { return centidegrees(m_rotation->get_angle()); }

    bool RotationSensor::getReversed() const { return m_rotation->get_reversed(); }

    std::int32_t RotationSensor::reverse() const { return m_rotation->reverse(); }

    std::int32_t RotationSensor::setDataRate(const milliseconds<std::uint8_t> rate)
    {
        return m_rotation->set_data_rate(rate.value());
    }
}