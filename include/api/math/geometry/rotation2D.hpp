// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <type_traits>
#include <utility>

#include "Eigen/Core"
#include "units/angle.h"
#include "units/core.h"

using namespace units::literals;

namespace aekulib
{

    /**
     * A rotation in a 2D coordinate frame represented by a point on the unit circle
     * (cosine and sine).
     *
     * The angle is continuous, that is if a Rotation2D is constructed with 361
     * degrees, it will return 361 degrees. This allows algorithms that wouldn't
     * want to see a discontinuity in the rotations as it sweeps past from 360 to 0
     * on the second time around.
     */
    class Rotation2D
    {
      public:
        /**
         * Constructs a Rotation2D with a default angle of 0 degrees.
         */
        constexpr Rotation2D() = default;

        /**
         * Constructs a Rotation2D with the given angle.
         *
         * @param value The value of the angle.
         */
        constexpr Rotation2D(units::radians<> ivalue) // NOLINT
            : value{ivalue}, cos{units::cos(ivalue)}, sin{units::sin(ivalue)}
        {}

        /**
         * Constructs a Rotation2D with the given x and y (cosine and sine)
         * components. The x and y don't have to be normalized.
         *
         * @param x The x component or cosine of the rotation.
         * @param y The y component or sine of the rotation.
         */
        constexpr Rotation2D(double x, double y)
        {
            double magnitude = hypot(x, y);
            if(magnitude > 1e-6)
            {
                cos = x / magnitude;
                sin = y / magnitude;
            }
            else
            {
                cos = 1.0;
                sin = 0.0;
            }

            value = units::radians{units::atan2(sin, cos)};
        }

        /**
         * Constructs a Rotation2D from a rotation matrix.
         *
         * @param rotationMatrix The rotation matrix.
         * @throws std::domain_error if the rotation matrix isn't special orthogonal.
         */
        constexpr explicit Rotation2D(const Eigen::Matrix2d &rotationMatrix)
        {
            auto impl = []<typename Matrix2d>(const Matrix2d &R) -> std::pair<double, double> {
                // Require that the rotation matrix is special orthogonal. This is true if
                // the matrix is orthogonal (RRᵀ = I) and normalized (determinant is 1).
                if((R * R.transpose() - Matrix2d::Identity()).norm() > 1e-9)
                {
                    throw std::domain_error("Rotation matrix isn't orthogonal");
                }
                // HACK: Uses ct_matrix instead of <Eigen/LU> for determinant because
                //       including <Eigen/LU> doubles compilation times on MSVC, even if
                //       this constructor is unused. MSVC's frontend inefficiently parses
                //       large headers; GCC and Clang are largely unaffected.
                if(fabs(R.determinant() - 1.0) > 1e-9)
                {
                    throw std::domain_error("Rotation matrix is orthogonal but not special orthogonal");
                }

                // R = [cosθ  −sinθ]
                //     [sinθ   cosθ]
                return {R(0, 0), R(1, 0)};
            };

            if(std::is_constant_evaluated())
            {
                auto cossin = impl(rotationMatrix);
                cos = std::get<0>(cossin);
                sin = std::get<1>(cossin);
            }
            else
            {
                auto cossin = impl(rotationMatrix);
                cos = std::get<0>(cossin);
                sin = std::get<1>(cossin);
            }

            value = atan2(sin, cos);
        }

        /**
         * Adds two rotations together, with the result being bounded between -π and
         * π.
         *
         * For example, <code>Rotation2D{30_deg} + Rotation2D{60_deg}</code> equals
         * <code>Rotation2D{units::radian_t{std::numbers::pi/2.0}}</code>
         *
         * @param other The rotation to add.
         *
         * @return The sum of the two rotations.
         */
        constexpr Rotation2D operator+(const Rotation2D &other) const { return rotateBy(other); }

        /**
         * Subtracts the new rotation from the current rotation and returns the new
         * rotation.
         *
         * For example, <code>Rotation2D{10_deg} - Rotation2D{100_deg}</code> equals
         * <code>Rotation2D{units::radian_t{-std::numbers::pi/2.0}}</code>
         *
         * @param other The rotation to subtract.
         *
         * @return The difference between the two rotations.
         */
        constexpr Rotation2D operator-(const Rotation2D &other) const { return *this + -other; }

        /**
         * Takes the inverse of the current rotation. This is simply the negative of
         * the current angular value.
         *
         * @return The inverse of the current rotation.
         */
        constexpr Rotation2D operator-() const { return Rotation2D{-value}; }

        /**
         * Multiplies the current rotation by a scalar.
         *
         * @param scalar The scalar.
         *
         * @return The new scaled Rotation2D.
         */
        constexpr Rotation2D operator*(double scalar) const { return Rotation2D{value * scalar}; }

        /**
         * Divides the current rotation by a scalar.
         *
         * @param scalar The scalar.
         *
         * @return The new scaled Rotation2D.
         */
        constexpr Rotation2D operator/(double scalar) const { return *this * (1.0 / scalar); }

        /**
         * Checks equality between this Rotation2D and another object.
         *
         * @param other The other object.
         * @return Whether the two objects are equal.
         */
        constexpr bool operator==(const Rotation2D &other) const
        {
            return hypot(getCos() - other.getCos(), getSin() - other.getSin()) < 1E-9;
        }

        /**
         * Adds the new rotation to the current rotation using a rotation matrix.
         *
         * <pre>
         * [cos_new]   [other.cos, -other.sin][cos]
         * [sin_new] = [other.sin,  other.cos][sin]
         * value_new = std::atan2(sin_new, cos_new)
         * </pre>
         *
         * @param other The rotation to rotate by.
         *
         * @return The new rotated Rotation2D.
         */
        constexpr Rotation2D rotateBy(const Rotation2D &other) const
        {
            return {getCos() * other.getCos() - getSin() * other.getSin(),
                    getCos() * other.getSin() + getSin() * other.getCos()};
        }

        /**
         * Returns matrix representation of this rotation.
         */
        constexpr Eigen::Matrix2d toMatrix() const
        {
            // R = [cosθ  −sinθ]
            //     [sinθ   cosθ]
            return Eigen::Matrix2d{{cos, -sin}, {sin, cos}};
        }

        /**
         * Returns the radian value of the rotation.
         *
         * @return The radian value of the rotation.
         * @see AngleModulus to constrain the angle within (-π, π]
         */
        constexpr units::radians<> radians() const { return value; }

        /**
         * Returns the degree value of the rotation.
         *
         * @return The degree value of the rotation.
         * @see InputModulus to constrain the angle within (-180, 180]
         */
        constexpr units::degrees<> degrees() const { return value; }

        /**
         * Returns the cosine of the rotation.
         *
         * @return The cosine of the rotation.
         */
        constexpr double getCos() const { return cos; }

        /**
         * Returns the sine of the rotation.
         *
         * @return The sine of the rotation.
         */
        constexpr double getSin() const { return sin; }

        /**
         * Returns the tangent of the rotation.
         *
         * @return The tangent of the rotation.
         */
        constexpr double getTan() const { return getSin() / getCos(); }

      private:
        units::radians<> value = 0_rad;
        units::dimensionless<> cos = 1;
        units::dimensionless<> sin = 0;
    };
} // namespace aekulib