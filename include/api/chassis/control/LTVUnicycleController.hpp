#pragma once

#include "units/angular_velocity.h"
#include "units/time.h"
#include "units/velocity.h"
#include "Eigen/Core"
#include "api/math/geometry/pose2D.hpp"
#include "api/math/discretization.hpp"
#include "api/solver/DARE.hpp"
#include <map>

using namespace units::literals;

namespace aekulib
{
    class LTVUnicycleController
    {
      public:
        LTVUnicycleController(const Pose2D &istateError, const units::milliseconds<> itickDelay,
                              const units::inches_per_second<> imaxVelocity,
                              const units::radians_per_second<> imaxAngular,
                              const units::inches_per_second<> iresolution = 0.01_ips);

        LTVUnicycleController(const Eigen::Matrix<double, 3, 3> Q, const Eigen::Matrix<double, 2, 2> R,
                              const units::milliseconds<> itickDelay,
                              const units::inches_per_second<> imaxVelocity,
                              const units::inches_per_second<> iresolution = 0.01_ips);

        bool atReference() const;

        std::pair<units::inches_per_second<>, units::radians_per_second<>>
        calculate(const Pose2D &ireferencePose, const Pose2D &icurrentPose,
                  const units::inches_per_second<> &ilinearVelocityReference,
                  const units::degrees_per_second<> &iangularVelocityReference);

      private:
        std::map<units::inches_per_second<>, Eigen::Matrix<double, 2, 3>> table;
    };
}