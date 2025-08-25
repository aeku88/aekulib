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
    inline Eigen::MatrixXd makeCostMatrix(const std::span<const double> costs)
    {
        Eigen::MatrixXd result{costs.size(), costs.size()};
        result.setZero();

        for(size_t i = 0; i < costs.size(); ++i)
        {
            if(costs[i] == std::numeric_limits<double>::infinity())
            {
                result(i, i) = 0.0;
            }
            else
            {
                result(i, i) = 1.0 / (std::pow(costs[i], 2));
            }
        }
        return result;
    }

    class LTVUnicycleController
    {
      public:
        LTVUnicycleController(const std::array<double, 3> &Qelems, const std::array<double, 2> &Relems,
                              const units::milliseconds<> itickDelay,
                              const units::inches_per_second<> imaxVelocity,
                              const units::inches_per_second<> iresolution = 0.01_ips);

        bool atReference() const;

        void setTolerance(const Pose2D &poseTolerance);

        std::pair<units::inches_per_second<>, units::radians_per_second<>>
        calculate(const Pose2D &ireferencePose, const Pose2D &icurrentPose,
                  const units::inches_per_second<> &ilinearVelocityReference,
                  const units::degrees_per_second<> &iangularVelocityReference);

      private:
        std::map<units::inches_per_second<>, Eigen::Matrix<double, 2, 3>> table;
        Pose2D poseError, poseTolerance;
    };
}