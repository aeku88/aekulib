#pragma once

#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/chassis/control/LTVUnicycleController.hpp"
#include "api/chassis/model/DDChassisModelIntegrated.hpp"
#include "api/devices/motorGroup.hpp"
#include "api/odometry/odometry.hpp"
#include "api/math/geometry/pose2D.hpp"
#include "squiggles/geometry/profilepoint.hpp"
#include "squiggles/spline.hpp"
#include <cstdint>

using namespace units;

namespace aekulib
{

    class ChassisController
    {
      public:
        ChassisController(const std::shared_ptr<LTVUnicycleController> &icontroller,
                          const std::shared_ptr<ChassisConfiguration> &iconfig,
                          const std::shared_ptr<ChassisSensors> &isensors,
                          const std::shared_ptr<Odometry> &iodometry,
                          const std::shared_ptr<MotorGroup> &ileft,
                          const std::shared_ptr<MotorGroup> &iright);

        void setTarget(const std::vector<Pose2D> &iwaypoints, const bool ireverse = false);

        void drive();

      private:
        std::shared_ptr<LTVUnicycleController> controller = nullptr;
        std::shared_ptr<ChassisKinematics> kinematics = nullptr;
        std::shared_ptr<ChassisConfiguration> config = nullptr;
        std::shared_ptr<ChassisSensors> sensors = nullptr;
        std::shared_ptr<DifferentialDriveChassisModelIntegrated> model = nullptr;
        std::shared_ptr<Odometry> odometry = nullptr;
        squiggles::SplineGenerator generator;
        std::vector<squiggles::ProfilePoint> path;
        bool reverse = false;
        Pose2D refPose;
    };
}