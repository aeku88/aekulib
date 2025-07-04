#include "api/devices/rotationSensor.hpp"

namespace aekulib
{
    RotationSensor::RotationSensor(const std::uint8_t port)
: m_rotation(std::make_shared<pros::Rotation>(port))
{
}

    RotationSensor::RotationSensor(const std::uint8_t port, const bool reverse)
: m_rotation(std::make_shared<pros::Rotation>(port, reverse))
{
}

    std::int32_t RotationSensor::reset()
{
 return m_rotation->reset();
}

    std::int32_t RotationSensor::resetPosition()
{
return m_rotation->resetPosition();
}

    std::int32_t RotationSensor::setReversed(const bool value)
{
return m_rotation->set_reversed(value);
}

    std::int32_t RotationSensor::setPosition(const degrees<> angle)
{
    return m_rotation->set_position(convert<centidegrees<>>(angle).value());
}

    degrees<> RotationSensor::getPosition() const
    {}

    degrees_per_second<> RotationSensor::getVelocity() const
    {}

    degrees<> RotationSensor::getAngle() const
    {}

    bool RotationSensor::getReversed() const
    {}

    std::int32_t RotationSensor::reverse() const
    {}

    std::int32_t RotationSensor::setDataRate(const milliseconds<std::uint8_t> rate)
    {}
}