#include "api/chassis/control/ramseteController.hpp"
#include <memory>

namespace aekulib
{
    RamseteController::RamseteController(const std::shared_ptr<RamseteGains> &iramseteGains,
                                         const std::shared_ptr<Odometry> &iodometry)
    {}
    void RamseteController::setTargetValues(inches<> x, inches<> y, degrees<> theta)
    {
        x_d = x;
        y_d = y;
        theta_d = theta;
    }

    void RamseteController::step()
    {
        inches<> wheel_circumference;
        inches<> x_current = odometry->getPosition()[0];
        inches<> y_current = odometry->getPosition()[1];
        degrees<> theta_current = odometry->getOrientation() * 180 / std::numbers::pi;
        double b = ramsetegains->getB();
        double zeta = ramsetegains->getZeta();
        // errors
        inches<> x_error = x_current - x_d;
        inches<> y_error = y_current - y_d;
        degrees<> theta_error = theta_current - theta_d;
        radians<> theta_error_rad = theta_error * std::numbers::pi / 180;
        // tuning desired values
        double smallScalar = 0.01;
        inches_per_second<> vD = inches_per_second(smallScalar * x_error.value());
        degrees_per_second<> omega_d = degrees_per_second(smallScalar * theta_error.value());
        // finding k term
        double k = 2.0 * zeta * std::sqrt(omega_d.value() * omega_d.value() + b * vD.value() * vD.value());
        // finding linear velocity
        v = inches_per_second(vD.value() * std::cos(theta_error_rad.value()))
            + inches_per_second<>(k * x_error.value());
        // finding angular velocity
        double omega_rad
          = (omega_d.value() + k * theta_error.value()
             + (b * vD.value() * std::sin(theta_error_rad.value()) * y_error.value()) / theta_error.value());
        omega = degrees_per_second<>(omega_rad * 180 / std::numbers::pi);
    }
    std::pair<inches_per_second<>, degrees_per_second<>> RamseteController::getVelocities()
    {
        return std::pair<inches_per_second<>, degrees_per_second<>>{v, omega};
    }
}