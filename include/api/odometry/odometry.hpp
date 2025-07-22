#pragma once
#include <cmath>
#include <numbers>
#include <vector>
#include "units/length.h"
#include "units/angle.h"
#include "pros/rtos.hpp"
#include "api/devices/rotationSensor.hpp"
#include "api/devices/imu.hpp"
#include "Eigen/Core"

using namespace units;
using namespace units::literals;
namespace aekulib
{
    class Odometry
    {
        //---------------------------------------------------------
        //                     IMPORTANT
        //---------------------------------------------------------
        // REMEMBER:
        // YOU NEED TO SET CONSTANTS FOR Tl, Tr, Ts, and wheel radius.
        // ALSO PUT CORRECT X AND Y INITIAL COORDINATES
      public:
        Odometry(uint8_t right_rotation_port, uint8_t back_rotation_port, uint8_t imu_port,
                 inches<> initial_x, inches<> initial_y, inches<> wheel_radius, inches<> tr, inches<> ts,
                 radians<> dir);

        Eigen::Vector2<inches<>> getPosition() const;

        inches<> getPositionChangeX() const;

        inches<> getPositionChangeY() const;

        radians<> getOrientation() const;

        radians<> getOrientationChange() const;

      private:
        void update();

        radians<> angle_change;

        void wheel_distance(inches<> &right_dist, inches<> &back_dist);

        aekulib::RotationSensor rotation_sensor_right;
        aekulib::RotationSensor rotation_sensor_back;

        aekulib::IMU inertial_sensor;
        radians<> inertial_heading_previous;

        // distance from center to tracking wheels
        const inches<> Tr;
        const inches<> Ts;
        const inches<> wheel_radius;

        // angle change and radius to calculate distance travelled by wheel
        radians<> wheel_angle_right_previous = 0_rad;
        radians<> wheel_angle_back_previous = 0_rad;

        // total wheel distances
        inches<> right_dist_total = 0_in;

        radians<> dir_initial;

        // global orientation
        radians<> orientation;

        // the global x and y coords
        inches<> x_coord;
        inches<> y_coord;

        // x and y change
        inches<> x_change_correct = 0_in;
        inches<> y_change_correct = 0_in;

        pros::Task update_task;
    };
}