// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "units/angle.h"
#include "units/length.h"

using namespace units::literals;

namespace aekulib
{

    /**
     * A change in distance along a 2D arc since the last pose update. We can use
     * ideas from differential calculus to create new Pose2ds from a Twist2d and
     * vice versa.
     *
     * A Twist can be used to represent a difference between two poses.
     */
    struct Twist2D
    {
        /**
         * Linear "dx" component
         */
        units::inches<> dx = 0_in;

        /**
         * Linear "dy" component
         */
        units::inches<> dy = 0_in;

        /**
         * Angular "dtheta" component (radians)
         */
        units::radians<> dtheta = 0_rad;

        /**
         * Checks equality between this Twist2d and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        constexpr bool operator==(const Twist2D &other) const
        {
            return units::abs(dx - other.dx) < 1E-9_in && units::abs(dy - other.dy) < 1E-9_in
                   && units::abs(dtheta - other.dtheta) < 1E-9_rad;
        }

        /**
         * Scale this by a given factor.
         *
         * @param factor The factor by which to scale.
         * @return The scaled Twist2d.
         */
        constexpr Twist2D operator*(double factor) const
        {
            return Twist2D{dx * factor, dy * factor, dtheta * factor};
        }
    };

}