#include "api/odometry/odometry.hpp"

namespace aekulib
{
    Eigen::Vector2<inches<>> Odometry::getPosition() const { return {x_coord, y_coord}; }

    inches<> Odometry::getPositionChangeX() const { return x_change_correct; }

    inches<> Odometry::getPositionChangeY() const { return y_change_correct; }

    radians<> Odometry::getOrientation() const { return orientation; }

    radians<> Odometry::getOrientationChange() const { return angle_change; }

    Odometry::Odometry(uint8_t right_rotation_port, uint8_t back_rotation_port, uint8_t imu_port,
                       inches<> initial_x, inches<> initial_y, inches<> wheel_radius, inches<> tr,
                       inches<> ts, radians<> dir)
        : rotation_sensor_right(right_rotation_port), rotation_sensor_back(back_rotation_port),
          inertial_sensor(imu_port), Tr(tr), Ts(ts), wheel_radius(wheel_radius), x_coord(initial_x),
          y_coord(initial_y), dir_initial(dir), update_task([this]() { this->update(); })
    {
        while(inertial_sensor.isCalibrating())
        {
            pros::delay(10);
        }

        inertial_heading_previous = radians(dir_initial);

        // Reset rotation sensors
        rotation_sensor_right.resetPosition();
        rotation_sensor_back.resetPosition();

        wheel_angle_right_previous = 0_rad;
        wheel_angle_back_previous = 0_rad;
    }

    void Odometry::wheel_distance(inches<> &right_dist, inches<> &back_dist)
    {
        // The wheel angle currently
        radians<> wheel_angle_right_current = rotation_sensor_right.getPosition();
        radians<> wheel_angle_back_current = rotation_sensor_back.getPosition();

        // distance travelled by each wheel based on variables above
        right_dist = wheel_radius * (wheel_angle_right_current - wheel_angle_right_previous).value();
        back_dist = wheel_radius * (wheel_angle_back_current - wheel_angle_back_previous).value();

        // The previous angle is changed to the new angle for the next cycle
        wheel_angle_right_previous = wheel_angle_right_current;
        wheel_angle_back_previous = wheel_angle_back_current;
    }

    void Odometry::update()
    {
        while(true)
        {
            radians<> current_inertial_heading = radians(dir_initial) - inertial_sensor.getRotation();

            angle_change = current_inertial_heading - inertial_heading_previous;

            inches<> right_distance, back_distance;
            wheel_distance(right_distance, back_distance);

            // calculate new global orientation
            orientation = radians(current_inertial_heading);
            normalizeAngle(orientation);

            x_change = (fabs(angle_change) < 1e-6_rad) ? back_distance
                                                       : 2 * std::sin(angle_change.value() / 2)
                                                           * (((back_distance) / angle_change.value()) + Ts);
            y_change = (fabs(angle_change) < 1e-6_rad) ? right_distance
                                                       : 2 * std::sin(angle_change.value() / 2)
                                                           * (((right_distance) / angle_change.value()) + Tr);

            // The next part is to change local coordinates to global coordinates.

            // calculate average orientation
            radians<> average_orientation = inertial_heading_previous + (angle_change / 2);

            // convert the local coordinates to polar coordinates
            inches<> radius = 1_in * std::sqrt(((x_change * x_change) + (y_change * y_change)).value());

            radians<> angle = std::atan2(y_change.value(), x_change.value()) * 1_rad;
            if(fabs(x_change.value()) < 1e-6 && fabs(y_change.value()) < 1e-6)
            {
                angle = 0_rad;
            }

            // converting polar coordinates back to cartesian coordinates
            x_change_correct = radius * std::cos(angle.value() - average_orientation.value());
            y_change_correct = radius * std::sin(angle.value() - average_orientation.value());

            x_coord += x_change_correct;
            y_coord += y_change_correct;

            inertial_heading_previous = current_inertial_heading;

            pros::delay(10);
        }
    }

    radians<> Odometry::normalizeAngle(radians<> angle)
    {
        double val = angle.value();
        val = std::fmod(val, 2.0 * std::numbers::pi_v<double>);
        if(val <= -std::numbers::pi_v<double>)
        {
            val += 2.0 * std::numbers::pi_v<double>;
        }
        if(val > std::numbers::pi_v<double>)
        {
            val -= 2.0 * std::numbers::pi_v<double>;
        }
        return radians(val);
    }
}
