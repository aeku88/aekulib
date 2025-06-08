#pragma once

#include "pros/rtos.hpp"
#include "units/time.h"
#include <vector>

using namespace units;
using namespace units::literals;

namespace aekulib
{
    struct PIDGains
    {
      public:
        PIDGains(const double ikP, const double ikI, const double ikD) : kP(ikP), kI(ikI), kD(ikD) {}

        std::vector<double> getGains() { return {kP, kI, kD}; }

      private:
        double kP, kI, kD;
    };

    template <class T> class PIDController
    {
      public:
        PIDController(const PIDGains &igains, const milliseconds<std::uint8_t> itickDelay = 10_ms);
        ~PIDController() { step_task.remove(); }

        void setTarget(const T isetpoint);

        T getOutput();

      private:
        void loop();

        void step();

        pros::Task step_task;

        PIDGains gains;
        T setpoint, processVariable, previousError, integral;

        milliseconds<double> tickDelay;
    };

    template <class T>
    PIDController<T>::PIDController(const PIDGains &igains, const milliseconds<std::uint8_t> itickDelay)
        : gains(igains), tickDelay(itickDelay)
    {
        step_task([this] {
            this->loop();
            pros::delay(10);
        });
    }

    template <class T> void PIDController<T>::setTarget(const T isetpoint) { setpoint = isetpoint; }

    template <class T> void PIDController<T>::loop()
    {
        while(true)
        {
            step();
            pros::delay(tickDelay.value());
        }
    }

    template <class T> void PIDController<T>::step()
    {
        T error = setpoint - processVariable;

        integral = error * tickDelay.value();
        auto derivative = (error - previousError) / tickDelay;

        processVariable = T(gains.getGains()[0] * error.value() + gains.getGains()[1] * integral.value()
                            + gains.getGains()[2] * derivative.value());

        previousError = error;
    }

    template <class T> T PIDController<T>::getOutput() { return processVariable; }
}
