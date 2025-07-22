// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <initializer_list>
#include <span>
#include <utility>

#include "api/math/geometry/rotation2D.hpp"
#include "api/math/geometry/translation2D.hpp"
#include "api/math/geometry/twist2D.hpp"
#include "units/length.h"

namespace aekulib
{
    class Transform2D;

    /**
     * Represents a 2D pose containing translational and rotational elements.
     */
    class Pose2D
    {
      public:
        /**
         * Constructs a pose at the origin facing toward the positive X axis.
         */
        constexpr Pose2D() = default;

        /**
         * Constructs a pose with the specified translation and rotation.
         *
         * @param itranslation The translational component of the pose.
         * @param irotation The rotational component of the pose.
         */
        constexpr Pose2D(Translation2D itranslation, Rotation2D irotation)
            : translation{std::move(itranslation)}, rotation{std::move(irotation)}
        {}

        /**
         * Constructs a pose with x and y translations instead of a separate
         * Translation2D.
         *
         * @param ix The x component of the translational component of the pose.
         * @param iy The y component of the translational component of the pose.
         * @param irotation The rotational component of the pose.
         */
        constexpr Pose2D(units::inches<> ix, units::inches<> iy, Rotation2D irotation)
            : translation{ix, iy}, rotation{std::move(irotation)}
        {}

        /**
         * Constructs a pose with the specified affine transformation matrix.
         *
         * @param matrix The affine transformation matrix.
         * @throws std::domain_error if the affine transformation matrix is invalid.
         */
        constexpr explicit Pose2D(const Eigen::Matrix3d &matrix)
            : translation{Eigen::Vector2d{{matrix(0, 2)}, {matrix(1, 2)}}},
              rotation{Eigen::Matrix2d{{matrix(0, 0), matrix(0, 1)}, {matrix(1, 0), matrix(1, 1)}}}
        {
            if(matrix(2, 0) != 0.0 || matrix(2, 1) != 0.0 || matrix(2, 2) != 1.0)
            {
                throw std::domain_error("Affine transformation matrix is invalid");
            }
        }

        /**
         * Transforms the pose by the given transformation and returns the new
         * transformed pose.
         *
         * <pre>
         * [x_new]    [cos, -sin, 0][transform.x]
         * [y_new] += [sin,  cos, 0][transform.y]
         * [t_new]    [  0,    0, 1][transform.t]
         * </pre>
         *
         * @param other The transform to transform the pose by.
         *
         * @return The transformed pose.
         */
        constexpr Pose2D operator+(const Transform2D &other) const { return transformBy(other); }

        /**
         * Returns the Transform2D that maps the one pose to another.
         *
         * @param other The initial pose of the transformation.
         * @return The transform that maps the other pose to the current pose.
         */
        constexpr Transform2D operator-(const Pose2D &other) const;

        /**
         * Checks equality between this Pose2D and another object.
         */
        constexpr bool operator==(const Pose2D &) const = default;

        /**
         * Returns the underlying translation.
         *
         * @return Reference to the translational component of the pose.
         */
        constexpr const Translation2D &getTranslation() const { return translation; }

        /**
         * Returns the X component of the pose's translation.
         *
         * @return The x component of the pose's translation.
         */
        constexpr units::inches<> getX() const { return translation.getX(); }

        /**
         * Returns the Y component of the pose's translation.
         *
         * @return The y component of the pose's translation.
         */
        constexpr units::inches<> getY() const { return translation.getY(); }

        /**
         * Returns the underlying rotation.
         *
         * @return Reference to the rotational component of the pose.
         */
        constexpr const Rotation2D &getRotation() const { return rotation; }

        /**
         * Multiplies the current pose by a scalar.
         *
         * @param scalar The scalar.
         *
         * @return The new scaled Pose2D.
         */
        constexpr Pose2D operator*(double scalar) const
        {
            return Pose2D{translation * scalar, rotation * scalar};
        }

        /**
         * Divides the current pose by a scalar.
         *
         * @param scalar The scalar.
         *
         * @return The new scaled Pose2D.
         */
        constexpr Pose2D operator/(double scalar) const { return *this * (1.0 / scalar); }

        /**
         * Rotates the pose around the origin and returns the new pose.
         *
         * @param other The rotation to transform the pose by.
         *
         * @return The rotated pose.
         */
        constexpr Pose2D rotateBy(const Rotation2D &other) const
        {
            return {translation.rotateBy(other), rotation.rotateBy(other)};
        }

        /**
         * Transforms the pose by the given transformation and returns the new pose.
         * See + operator for the matrix multiplication performed.
         *
         * @param other The transform to transform the pose by.
         *
         * @return The transformed pose.
         */
        constexpr Pose2D transformBy(const Transform2D &other) const;

        /**
         * Returns the current pose relative to the given pose.
         *
         * This function can often be used for trajectory tracking or pose
         * stabilization algorithms to get the error between the reference and the
         * current pose.
         *
         * @param other The pose that is the origin of the new coordinate frame that
         * the current pose will be converted into.
         *
         * @return The current pose relative to the new origin pose.
         */
        constexpr Pose2D relativeTo(const Pose2D &other) const;

        /**
         * Rotates the current pose around a point in 2D space.
         *
         * @param point The point in 2D space to rotate around.
         * @param rot The rotation to rotate the pose by.
         *
         * @return The new rotated pose.
         */
        constexpr Pose2D rotateAround(const Translation2D &point, const Rotation2D &rot) const
        {
            return {translation.rotateAround(point, rot), rotation.rotateBy(rot)};
        }

        /**
         * Obtain a new Pose2D from a (constant curvature) velocity.
         *
         * See https://file.tavsys.net/control/controls-engineering-in-frc.pdf section
         * 10.2 "Pose exponential" for a derivation.
         *
         * The twist is a change in pose in the robot's coordinate frame since the
         * previous pose update. When the user runs exp() on the previous known
         * field-relative pose with the argument being the twist, the user will
         * receive the new field-relative pose.
         *
         * "Exp" represents the pose exponential, which is solving a differential
         * equation moving the pose forward in time.
         *
         * @param twist The change in pose in the robot's coordinate frame since the
         * previous pose update. For example, if a non-holonomic robot moves forward
         * 0.01 meters and changes angle by 0.5 degrees since the previous pose
         * update, the twist would be Twist2D{0.01_m, 0_m, 0.5_deg}.
         *
         * @return The new pose of the robot.
         */
        constexpr Pose2D exp(const Twist2D &twist) const;

        /**
         * Returns a Twist2D that maps this pose to the end pose. If c is the output
         * of a.Log(b), then a.Exp(c) would yield b.
         *
         * @param end The end pose for the transformation.
         *
         * @return The twist that maps this to end.
         */
        constexpr Twist2D log(const Pose2D &end) const;

        /**
         * Returns an affine transformation matrix representation of this pose.
         */
        constexpr Eigen::Matrix3d toMatrix() const
        {
            auto vec = translation.toVector();
            auto mat = rotation.toMatrix();
            return Eigen::Matrix3d{
              {mat(0, 0), mat(0, 1), vec(0)}, {mat(1, 0), mat(1, 1), vec(1)}, {0.0, 0.0, 1.0}};
        }

        /**
         * Returns the nearest Pose2D from a collection of poses.
         *
         * If two or more poses in the collection have the same distance from this
         * pose, return the one with the closest rotation component.
         *
         * @param poses The collection of poses.
         * @return The nearest Pose2D from the collection.
         */
        constexpr Pose2D nearest(std::span<const Pose2D> poses) const
        {
            return *std::min_element(poses.begin(), poses.end(), [this](const Pose2D &a, const Pose2D &b) {
                auto aDistance = this->getTranslation().distance(a.getTranslation());
                auto bDistance = this->getTranslation().distance(b.getTranslation());

                // If the distances are equal sort by difference in rotation
                if(aDistance == bDistance)
                {
                    return fabs((this->getRotation() - a.getRotation()).radians().value())
                           < fabs((this->getRotation() - b.getRotation()).radians().value());
                }
                return aDistance < bDistance;
            });
        }

        /**
         * Returns the nearest Pose2D from a collection of poses.
         *
         * If two or more poses in the collection have the same distance from this
         * pose, return the one with the closest rotation component.
         *
         * @param poses The collection of poses.
         * @return The nearest Pose2D from the collection.
         */
        constexpr Pose2D Nearest(std::initializer_list<Pose2D> poses) const
        {
            return *std::min_element(poses.begin(), poses.end(), [this](const Pose2D &a, const Pose2D &b) {
                auto aDistance = this->getTranslation().distance(a.getTranslation());
                auto bDistance = this->getTranslation().distance(b.getTranslation());

                // If the distances are equal sort by difference in rotation
                if(aDistance == bDistance)
                {
                    return fabs((this->getRotation() - a.getRotation()).radians().value())
                           < fabs((this->getRotation() - b.getRotation()).radians().value());
                }
                return aDistance < bDistance;
            });
        }

      private:
        Translation2D translation;
        Rotation2D rotation;
    };

}

#include "api/math/geometry/transform2D.hpp"

namespace aekulib
{

    constexpr Transform2D Pose2D::operator-(const Pose2D &other) const
    {
        const auto pose = this->relativeTo(other);
        return Transform2D{pose.getTranslation(), pose.getRotation()};
    }

    constexpr Pose2D Pose2D::transformBy(const Transform2D &other) const
    {
        return {translation + (other.Translation().rotateBy(rotation)), other.getRotation() + rotation};
    }

    constexpr Pose2D Pose2D::relativeTo(const Pose2D &other) const
    {
        const Transform2D transform{other, *this};
        return {transform.Translation(), transform.getRotation()};
    }

    constexpr Pose2D Pose2D::exp(const Twist2D &twist) const
    {
        const auto dx = twist.dx;
        const auto dy = twist.dy;
        const auto dtheta = twist.dtheta.value();

        const auto sinTheta = sin(dtheta);
        const auto cosTheta = cos(dtheta);

        double s, c;
        if(fabs(dtheta) < 1E-9)
        {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        }
        else
        {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }

        const Transform2D transform{Translation2D{dx * s - dy * c, dx * c + dy * s},
                                    Rotation2D{cosTheta, sinTheta}};

        return *this + transform;
    }

    constexpr Twist2D Pose2D::log(const Pose2D &end) const
    {
        const auto transform = end.relativeTo(*this);
        const auto dtheta = transform.getRotation().radians().value();
        const auto halfDtheta = dtheta / 2.0;

        const auto cosMinusOne = transform.getRotation().getCos() - 1;

        double halfThetaByTanOfHalfDtheta;

        if(fabs(cosMinusOne) < 1E-9)
        {
            halfThetaByTanOfHalfDtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else
        {
            halfThetaByTanOfHalfDtheta = -(halfDtheta * transform.getRotation().getSin()) / cosMinusOne;
        }

        const Translation2D translationPart
          = transform.getTranslation().rotateBy({halfThetaByTanOfHalfDtheta, -halfDtheta})
            * hypot(halfThetaByTanOfHalfDtheta, halfDtheta);

        return {translationPart.getX(), translationPart.getY(), units::radians<>{dtheta}};
    }
}