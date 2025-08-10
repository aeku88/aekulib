// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <utility>

#include "api/math/geometry/rotation2D.hpp"
#include "api/math/geometry/translation2D.hpp"

namespace aekulib
{
    class Pose2D;

    /**
     * Represents a transformation for a Pose2D in the pose's frame.
     */
    class Transform2D
    {
      public:
        /**
         * Constructs the transform that maps the initial pose to the final pose.
         *
         * @param initial The initial pose for the transformation.
         * @param final The final pose for the transformation.
         */
        constexpr Transform2D(const Pose2D &initial, const Pose2D &final);

        /**
         * Constructs a transform with the given translation and rotation components.
         *
         * @param translation Translational component of the transform.
         * @param rotation Rotational component of the transform.
         */
        constexpr Transform2D(Translation2D translation, Rotation2D rotation)
            : translation{std::move(translation)}, rotation{std::move(rotation)}
        {}

        /**
         * Constructs a transform with x and y translations instead of a separate
         * Translation2D.
         *
         * @param x The x component of the translational component of the transform.
         * @param y The y component of the translational component of the transform.
         * @param rotation The rotational component of the transform.
         */
        constexpr Transform2D(units::inches<> x, units::inches<> y, Rotation2D rotation)
            : translation{x, y}, rotation{std::move(rotation)}
        {}

        /**
         * Constructs a pose with the specified affine transformation matrix.
         *
         * @param matrix The affine transformation matrix.
         * @throws std::domain_error if the affine transformation matrix is invalid.
         */
        constexpr explicit Transform2D(const Eigen::Matrix3d &matrix)
            : translation{Eigen::Vector2d{{matrix(0, 2)}, {matrix(1, 2)}}},
              rotation{Eigen::Matrix2d{{matrix(0, 0), matrix(0, 1)}, {matrix(1, 0), matrix(1, 1)}}}
        {
            if(matrix(2, 0) != 0.0 || matrix(2, 1) != 0.0 || matrix(2, 2) != 1.0)
            {
                throw std::domain_error("Affine transformation matrix is invalid");
            }
        }

        /**
         * Constructs the identity transform -- maps an initial pose to itself.
         */
        constexpr Transform2D() = default;

        /**
         * Returns the translation component of the transformation.
         *
         * @return Reference to the translational component of the transform.
         */
        constexpr const Translation2D &Translation() const { return translation; }

        /**
         * Returns the X component of the transformation's translation.
         *
         * @return The x component of the transformation's translation.
         */
        constexpr units::inches<> getX() const { return translation.getX(); }

        /**
         * Returns the Y component of the transformation's translation.
         *
         * @return The y component of the transformation's translation.
         */
        constexpr units::inches<> getY() const { return translation.getY(); }

        /**
         * Returns an affine transformation matrix representation of this
         * transformation.
         */
        constexpr Eigen::Matrix3d toMatrix() const
        {
            auto vec = translation.toVector();
            auto mat = rotation.toMatrix();
            return Eigen::Matrix3d{
              {mat(0, 0), mat(0, 1), vec(0)}, {mat(1, 0), mat(1, 1), vec(1)}, {0.0, 0.0, 1.0}};
        }

        /**
         * Returns the rotational component of the transformation.
         *
         * @return Reference to the rotational component of the transform.
         */
        constexpr const Rotation2D &getRotation() const { return rotation; }

        /**
         * Invert the transformation. This is useful for undoing a transformation.
         *
         * @return The inverted transformation.
         */
        constexpr Transform2D inverse() const
        {
            // We are rotating the difference between the translations
            // using a clockwise rotation matrix. This transforms the global
            // delta into a local delta (relative to the initial pose).
            return Transform2D{(-Translation()).rotateBy(-getRotation()), -getRotation()};
        }

        /**
         * Multiplies the transform by the scalar.
         *
         * @param scalar The scalar.
         * @return The scaled Transform2D.
         */
        constexpr Transform2D operator*(double scalar) const
        {
            return Transform2D(translation * scalar, rotation * scalar);
        }

        /**
         * Divides the transform by the scalar.
         *
         * @param scalar The scalar.
         * @return The scaled Transform2D.
         */
        constexpr Transform2D operator/(double scalar) const { return *this * (1.0 / scalar); }

        /**
         * Composes two transformations. The second transform is applied relative to
         * the orientation of the first.
         *
         * @param other The transform to compose with this one.
         * @return The composition of the two transformations.
         */
        constexpr Transform2D operator+(const Transform2D &other) const;

        /**
         * Checks equality between this Transform2D and another object.
         */
        constexpr bool operator==(const Transform2D &) const = default;

      private:
        Translation2D translation;
        Rotation2D rotation;
    };

} // namespace frc

#include "api/math/geometry/pose2D.hpp"

namespace aekulib
{

    constexpr Transform2D::Transform2D(const Pose2D &initial, const Pose2D &final)
    {
        // We are rotating the difference between the translations
        // using a clockwise rotation matrix. This transforms the global
        // delta into a local delta (relative to the initial pose).
        translation = (final.getTranslation() - initial.getTranslation()).rotateBy(-initial.getRotation());

        rotation = final.getRotation() - initial.getRotation();
    }

    constexpr Transform2D Transform2D::operator+(const Transform2D &other) const
    {
        return Transform2D{Pose2D{}, Pose2D{}.transformBy(*this).transformBy(other)};
    }

}