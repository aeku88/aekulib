// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <initializer_list>
#include <span>

#include "Eigen/Core"

#include "api/math/geometry/rotation2D.hpp"
#include "units/length.h"

namespace aekulib
{

    /**
     * Represents a translation in 2D space.
     * This object can be used to represent a point or a vector.
     *
     * This assumes that you are using conventional mathematical axes.
     * When the robot is at the origin facing in the positive X direction, forward
     * is positive X and left is positive Y.
     */
    class Translation2D
    {
      public:
        /**
         * Constructs a Translation2D with X and Y components equal to zero.
         */
        constexpr Translation2D() = default;

        /**
         * Constructs a Translation2D with the X and Y components equal to the
         * provided values.
         *
         * @param x The x component of the translation.
         * @param y The y component of the translation.
         */
        constexpr Translation2D(units::inches<> ix, units::inches<> iy) : x{ix}, y{iy} {}

        /**
         * Constructs a Translation2D with the provided distance and angle. This is
         * essentially converting from polar coordinates to Cartesian coordinates.
         *
         * @param distance The distance from the origin to the end of the translation.
         * @param angle The angle between the x-axis and the translation vector.
         */
        constexpr Translation2D(units::inches<> distance, const Rotation2D &angle)
            : x{distance * angle.getCos()}, y{distance * angle.getSin()}
        {}

        /**
         * Constructs a Translation2D from a 2D translation vector. The values are
         * assumed to be in meters.
         *
         * @param vector The translation vector.
         */
        constexpr explicit Translation2D(const Eigen::Vector2d &vector)
            : x{units::inches<>{vector.x()}}, y{units::inches<>{vector.y()}}
        {}

        /**
         * Calculates the distance between two translations in 2D space.
         *
         * The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
         *
         * @param other The translation to compute the distance to.
         *
         * @return The distance between the two translations.
         */
        constexpr units::inches<> distance(const Translation2D &other) const
        {
            return units::hypot(other.getX() - x, other.getY() - y);
        }

        /**
         * Returns the X component of the translation.
         *
         * @return The X component of the translation.
         */
        constexpr units::inches<> getX() const { return x; }

        /**
         * Returns the Y component of the translation.
         *
         * @return The Y component of the translation.
         */
        constexpr units::inches<> getY() const { return y; }

        /**
         * Returns a 2D translation vector representation of this translation.
         *
         * @return A 2D translation vector representation of this translation.
         */
        constexpr Eigen::Vector2d toVector() const { return Eigen::Vector2d{{x.value(), y.value()}}; }

        /**
         * Returns the norm, or distance from the origin to the translation.
         *
         * @return The norm of the translation.
         */
        constexpr units::inches<> norm() const { return units::hypot(x, y); }

        /**
         * Returns the angle this translation forms with the positive X axis.
         *
         * @return The angle of the translation
         */
        constexpr Rotation2D Angle() const { return Rotation2D{x.value(), y.value()}; }

        /**
         * Applies a rotation to the translation in 2D space.
         *
         * This multiplies the translation vector by a counterclockwise rotation
         * matrix of the given angle.
         *
         * <pre>
         * [x_new]   [other.cos, -other.sin][x]
         * [y_new] = [other.sin,  other.cos][y]
         * </pre>
         *
         * For example, rotating a Translation2D of &lt;2, 0&gt; by 90 degrees will
         * return a Translation2D of &lt;0, 2&gt;.
         *
         * @param other The rotation to rotate the translation by.
         *
         * @return The new rotated translation.
         */
        constexpr Translation2D rotateBy(const Rotation2D &other) const
        {
            return {x * other.getCos() - y * other.getSin(), x * other.getSin() + y * other.getCos()};
        }

        /**
         * Rotates this translation around another translation in 2D space.
         *
         * <pre>
         * [x_new]   [rot.cos, -rot.sin][x - other.x]   [other.x]
         * [y_new] = [rot.sin,  rot.cos][y - other.y] + [other.y]
         * </pre>
         *
         * @param other The other translation to rotate around.
         * @param rot The rotation to rotate the translation by.
         * @return The new rotated translation.
         */
        constexpr Translation2D rotateAround(const Translation2D &other, const Rotation2D &rot) const
        {
            return {(x - other.getX()) * rot.getCos() - (y - other.getY()) * rot.getSin() + other.getX(),
                    (x - other.getX()) * rot.getSin() + (y - other.getY()) * rot.getCos() + other.getY()};
        }

        /**
         * Returns the sum of two translations in 2D space.
         *
         * For example, Translation3d{1.0, 2.5} + Translation3d{2.0, 5.5} =
         * Translation3d{3.0, 8.0}.
         *
         * @param other The translation to add.
         *
         * @return The sum of the translations.
         */
        constexpr Translation2D operator+(const Translation2D &other) const
        {
            return {x + other.getX(), y + other.getY()};
        }

        /**
         * Returns the difference between two translations.
         *
         * For example, Translation2D{5.0, 4.0} - Translation2D{1.0, 2.0} =
         * Translation2D{4.0, 2.0}.
         *
         * @param other The translation to subtract.
         *
         * @return The difference between the two translations.
         */
        constexpr Translation2D operator-(const Translation2D &other) const { return *this + -other; }

        /**
         * Returns the inverse of the current translation. This is equivalent to
         * rotating by 180 degrees, flipping the point over both axes, or negating all
         * components of the translation.
         *
         * @return The inverse of the current translation.
         */
        constexpr Translation2D operator-() const { return {-x, -y}; }

        /**
         * Returns the translation multiplied by a scalar.
         *
         * For example, Translation2D{2.0, 2.5} * 2 = Translation2D{4.0, 5.0}.
         *
         * @param scalar The scalar to multiply by.
         *
         * @return The scaled translation.
         */
        constexpr Translation2D operator*(double scalar) const { return {scalar * x, scalar * y}; }

        /**
         * Returns the translation divided by a scalar.
         *
         * For example, Translation2D{2.0, 2.5} / 2 = Translation2D{1.0, 1.25}.
         *
         * @param scalar The scalar to divide by.
         *
         * @return The scaled translation.
         */
        constexpr Translation2D operator/(double scalar) const { return operator*(1.0 / scalar); }

        /**
         * Checks equality between this Translation2D and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        constexpr bool operator==(const Translation2D &other) const
        {
            return units::abs(x - other.x) < 1E-9_in && units::abs(y - other.y) < 1E-9_in;
        }

        /**
         * Returns the nearest Translation2D from a collection of translations
         * @param translations The collection of translations.
         * @return The nearest Translation2D from the collection.
         */
        constexpr Translation2D Nearest(std::span<const Translation2D> translations) const
        {
            return *std::min_element(translations.begin(), translations.end(),
                                     [this](const Translation2D &a, const Translation2D &b) {
                                         return this->distance(a) < this->distance(b);
                                     });
        }

        /**
         * Returns the nearest Translation2D from a collection of translations
         * @param translations The collection of translations.
         * @return The nearest Translation2D from the collection.
         */
        constexpr Translation2D Nearest(std::initializer_list<Translation2D> translations) const
        {
            return *std::min_element(translations.begin(), translations.end(),
                                     [this](const Translation2D &a, const Translation2D &b) {
                                         return this->distance(a) < this->distance(b);
                                     });
        }

      private:
        units::inches<> x = 0_in;
        units::inches<> y = 0_in;
    };

}