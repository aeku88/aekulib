#pragma once

#include "api/devices/motor.hpp"
#include "pros/abstract_motor.hpp"
#include "units/magnetic_flux.h"
#include "units/torque.h"
#include <fstream>
#include <cstdio>

using namespace units::literals;

namespace aekulib
{
    inline void motorKt()
    {
        auto time = 0_ms;
        webers<> maxKt = webers<>(-10000000);
        Motor m(1, pros::v5::MotorGears::blue);
        m.move(12_V);

        while(time < 1_s)
        {
            auto kt = m.getTorque() / m.getCurrentDraw();
            maxKt = max(kt, maxKt);
            time += 10_ms;
            pros::delay(10);
        }

        std::ofstream outputFile("data.csv");
        outputFile << maxKt << '\n';
        outputFile.close();
    }

    inline void motorR()
    {
        Motor m(1, pros::v5::MotorGears::blue);
        m.move(12_V);

        while(true)
        {
            auto r = m.getVoltage() / m.getCurrentDraw();
            printf("%f %f %f\n", m.getVoltage().to<double>(), m.getCurrentDraw().to<double>(),
                   r.to<double>());

            pros::delay(10);
        }
    }
}