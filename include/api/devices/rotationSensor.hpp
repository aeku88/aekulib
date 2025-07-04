#pragma once

#include "units/angle.h"
#include "units/angular_velocity.h"

using namespace units;

namespace aekulib
{
    class RotationSensor
    {
public:
    explicit RotationSensor(const std::uint8_t port);

    explicit RotationSensor(const std::uint8_t port, const bool reverse);

    std::int32_t reset();

    std::int32_t resetPosition();

    std::int32_t setReversed(const bool value);

    std::int32_t setPosition(const degrees<> angle);

    degrees<> getPosition() const;

    degrees_per_second<> getVelocity() const;

        degrees<> getAngle() const;

    bool getReversed() const;

    std::int32_t reverse() const;

    std::int32_t setDataRate(const milliseconds<std::uint8_t> rate);
private:
    std::shared_ptr<pros::Rotation> m_rotation;
}
}
