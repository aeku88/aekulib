#include "api/chassis/control/LTVUnicycleController.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace aekulib
{
    LTVUnicycleController::LTVUnicycleController(const Eigen::Matrix<double, 3, 3> iQ,
                                                 const Eigen::Matrix<double, 2, 2> iR,
                                                 const units::milliseconds<> itickDelay,
                                                 const units::inches_per_second<> imaxVelocity,
                                                 const units::inches_per_second<> iresolution)
    {
        Eigen::Matrix<double, 3, 3> A = Eigen::Matrix<double, 3, 3>::Zero();
        Eigen::Matrix<double, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};

        for(auto velocity = -imaxVelocity; velocity < imaxVelocity; velocity += iresolution)
        {
            // The DARE is ill-conditioned if the velocity is close to zero, so don't
            // let the system stop.
            if(units::abs(velocity) < 1e-4_mps)
                A(1, 2) = 1e-4;
            else
                A(1, 2) = velocity.value();

            Eigen::Matrix<double, 3, 3> discA;
            Eigen::Matrix<double, 3, 2> discB;

            discretizeAB(A, B, itickDelay, &discA, &discB);

            auto S = DARE(discA, discB, iQ, iR);

            // K = (BᵀSB + R)⁻¹BᵀSA
            table.emplace(velocity,
                          (discB.transpose() * S * discB + iR).llt().solve(discB.transpose() * S * discA));
        }
    }

    std::pair<units::inches_per_second<>, units::radians_per_second<>>
    LTVUnicycleController::calculate(const Pose2D &ireferencePose, const Pose2D &icurrentPose,
                                     const units::inches_per_second<> &ilinearVelocityReference,
                                     const units::degrees_per_second<> &iangularVelocityReference)
    {
        auto poseError = ireferencePose.relativeTo(icurrentPose);

        const auto &K = table[ilinearVelocityReference];
        Eigen::Vector3d e{poseError.getX().value(), poseError.getY().value(),
                          poseError.getRotation().radians().value()};
        Eigen::Vector2d u = K * e;

        return {ilinearVelocityReference + units::inches_per_second<>{u(0)},
                iangularVelocityReference + units::radians_per_second<>{u(1)}};
    }
}
