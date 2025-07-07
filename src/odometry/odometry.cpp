#include "api/odometry/odometry.hpp"

namespace aekulib
{
    Eigen::Vector2<inches<>> Odometry::getPosition() const { return {x_coord, y_coord}; }

    degrees<> Odometry::getOrientation() const { return orientation; }

    void Odometry::wheel_distance(inches<> &left_dist, inches<> &right_dist, inches<> &back_dist)
    {
        const inches<> wheel_radius = 3_in;

        // Functions to initialize rotation sensors
        pros::Rotation rotation_sensor_1(1);
        pros::Rotation rotation_sensor_2(2);
        pros::Rotation rotation_sensor_3(3);

        // The wheel angle currently
        double wheel_angle_left_after = std::numbers::pi * rotation_sensor_1.get_position() / 18000;
        double wheel_angle_right_after = std::numbers::pi * rotation_sensor_2.get_position() / 18000;
        double wheel_angle_back_after = std::numbers::pi * rotation_sensor_3.get_position() / 18000;

        // distance travelled by each wheel based on variables above
        left_dist = wheel_radius * (wheel_angle_left_after - wheel_angle_left_previous);
        right_dist = wheel_radius * (wheel_angle_right_after - wheel_angle_right_previous);
        back_dist = wheel_radius * (wheel_angle_back_after - wheel_angle_back_previous);

        left_dist_total = wheel_radius * wheel_angle_left_after;
        right_dist_total = wheel_radius * wheel_angle_right_after;

        // The previous angle is changed to the new angle for the next cycle
        wheel_angle_left_previous = wheel_angle_left_after;
        wheel_angle_right_previous = wheel_angle_right_after;
        wheel_angle_back_previous = wheel_angle_back_after;
    }

    void Odometry::update()
    {
        while(true)
        {
            inches<> left_distance, right_distance, back_distance;
            wheel_distance(left_distance, right_distance, back_distance);

            // calculate new global orientation
            orientation = (left_dist_total - right_dist_total) / (Tl + Tr);

            double angle_change = (left_distance - right_distance) / (Tl + Tr);

            inches<> x_change, y_change;

            x_change = (angle_change == 0)
                         ? back_distance
                         : 2 * std::sin(angle_change / 2) * (((back_distance) / angle_change) + Ts);
            y_change = (angle_change == 0)
                         ? right_distance
                         : 2 * std::sin(angle_change / 2) * (((right_distance) / angle_change) + Tr);

            // The next part is to change local coordinates to global coordinates.

            // calculate average orientation
            double average_orientation = orientation - (angle_change / 2);

            // convert the local coordinates to polar coordinates
            inches<> radius = units::sqrt(x_change * x_change + y_change * y_change);
            double angle = std::atan2(y_change.value(), x_change.value());

            // converting polar coordinates back to cartesian coordinates
            inches<> x_change_correct = radius * std::cos(angle - average_orientation);
            inches<> y_change_correct = radius * std::sin(angle - average_orientation);

            x_coord += x_change_correct;
            y_coord += y_change_correct;

            pros::delay(10);
        }
    }
}
