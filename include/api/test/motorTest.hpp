#pragma once

#include "api/devices/motor.hpp"
#include "pros/abstract_motor.hpp"

using namespace units::literals;

namespace aekulib
{
    inline void motorKt()
    {
        Motor m(1, pros::v5::MotorGears::blue);
        m.move(12_V);

        while(true)
        {
            auto kt = m.getTorque() / m.getCurrentDraw();
            printf("%f %f %f\n", m.getTorque().value(), m.getCurrentDraw().value(), kt.value());

            pros::delay(10);
        }
    }

    inline void motorR()
    {
        Motor m(1, pros::v5::MotorGears::blue);
        m.move(12_V);

        while(true)
        {
            auto r = m.getVoltage() / m.getCurrentDraw();
            printf("%f %f %f\n", m.getVoltage().value(), m.getCurrentDraw().value(), r.value());

            pros::delay(10);
        }
    }
}