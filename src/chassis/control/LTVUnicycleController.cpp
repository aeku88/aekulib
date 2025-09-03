#include "api/chassis/control/LTVUnicycleController.hpp"
namespace aekulib
{
    LTVUnicycleController::LTVUnicycleController(const std::array<double, 3> &Qelems,
                                                 const std::array<double, 2> &Relems,
                                                 const units::milliseconds<> itickDelay,
                                                 const units::inches_per_second<> imaxVelocity,
                                                 const units::inches_per_second<> iresolution)
    {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Matrix<double, 3, 2> B{{1.0, 0.0}, {0.0, 0.0}, {0.0, 1.0}};
        Eigen::Matrix3d Q = makeCostMatrix(Qelems);
        Eigen::Matrix2d R = makeCostMatrix(Relems);

        for(auto velocity = -imaxVelocity; velocity < imaxVelocity; velocity += iresolution)
        {
            // The DARE is ill-conditioned if the velocity is close to zero, so don't
            // let the system stop.
            if(units::abs(velocity) < 1e-4_ips)
                A(1, 2) = 1e-4;
            else
                A(1, 2) = velocity.value();

            Eigen::Matrix<double, 3, 3> discA;
            Eigen::Matrix<double, 3, 2> discB;

            discretizeAB(A, B, itickDelay, &discA, &discB);

            auto S = DARE(discA, discB, Q, R);

            // K = (BᵀSB + R)⁻¹BᵀSA
            table.insert(velocity,
                         (discB.transpose() * S * discB + R).llt().solve(discB.transpose() * S * discA));
        }
    }

    std::pair<units::inches_per_second<>, units::radians_per_second<>>
    LTVUnicycleController::calculate(const Pose2D &ireferencePose, const Pose2D &icurrentPose,
                                     const units::inches_per_second<> &ilinearVelocityReference,
                                     const units::degrees_per_second<> &iangularVelocityReference)
    {
        poseError = ireferencePose.relativeTo(icurrentPose);

        const auto &K = table[ilinearVelocityReference];
        Eigen::Vector3d e{poseError.getX().value(), poseError.getY().value(),
                          poseError.getRotation().radians().value()};
        Eigen::Vector2d u = K * e;

        return {ilinearVelocityReference + units::inches_per_second<>{u(0)},
                iangularVelocityReference + units::radians_per_second<>{u(1)}};
    }
    void LTVUnicycleController::setTolerance(const Pose2D &poseTolerance)
    {
        this->poseTolerance = poseTolerance;
    }
    bool LTVUnicycleController::atReference() const
    {
        const auto &eTranslate = poseError.getTranslation();
        const auto &eRotate = poseError.getRotation();
        const auto &tolTranslate = poseTolerance.getTranslation();
        const auto &tolRotate = poseTolerance.getRotation();
        return abs(eTranslate.getX()) < tolTranslate.getX() && abs(eTranslate.getY()) < tolTranslate.getY()
               && abs(eRotate.radians()) < tolRotate.radians();
    }
}
