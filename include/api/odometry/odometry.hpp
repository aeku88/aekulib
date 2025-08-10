#pragma once

#include "api/chassis/chassisConfiguration.hpp"
#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/math/geometry/pose2D.hpp"
#include "api/math/geometry/rotation2D.hpp"

using namespace units;
using namespace units::literals;

namespace aekulib
{
    /*
     * Implements pose exponential odometry from
     * https://file.tavsys.net/control/controls-engineering-in-frc.pdf
     */
    class Odometry
    {
      public:
        Odometry(const std::shared_ptr<ChassisSensors> &isensors,
                 const std::shared_ptr<ChassisKinematics> &ikinematics);

        Pose2D update();

        inline Pose2D getPose() const { return pose; }

      private:
        std::shared_ptr<ChassisSensors> sensors = nullptr;

        std::shared_ptr<ChassisKinematics> kinematics = nullptr;

        Eigen::Vector2<inches<>> lastSensorVals = {0, 0};

        Rotation2D lastOrientation = {0_rad};

        Pose2D pose;
    };
}