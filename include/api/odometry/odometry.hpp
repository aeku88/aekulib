#pragma once
// #include <cmath>
#include <numbers>
#include <vector>
#include "units/length.h"
#include "units/angle.h"
#include "pros/rtos.hpp"
#include "api/devices/rotationSensor.hpp"
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
      public:
        Odometry();

        Eigen::Vector2<inches<>> getPosition() const;

        degrees<> getOrientation() const;

      private:
        void update();

        void wheel_distance(inches<> &left_dist, inches<> &right_dist, inches<> &back_dist);

        // distance from center to tracking wheels
        const inches<> Tl = 3_in;
        const inches<> Tr = 3_in;
        const inches<> Ts = 3_in;

        // angle change and radius to calculate distance travelled by wheel
        radians<> wheel_angle_left_previous = 0_rad;
        radians<> wheel_angle_right_previous = 0_rad;
        radians<> wheel_angle_back_previous = 0_rad;

        // total wheel distances
        inches<> left_dist_total = 0_in;
        inches<> right_dist_total = 0_in;

        // global orientation
        radians<> orientation = 0_rad;

        // the global x and y coords
        inches<> x_coord = 0_in;
        inches<> y_coord = 0_in;
    };
}