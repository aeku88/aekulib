#pragma once

#include "Eigen/Core"
#include "units/time.h"
#include "unsupported/Eigen/MatrixFunctions"

using namespace Eigen;

namespace aekulib
{
    /**
     * Discretizes the given continuous A matrix.
     *
     * @tparam States Number of states.
     * @param contA Continuous system matrix.
     * @param dt    Discretization timestep.
     * @param discA Storage for discrete system matrix.
     */
    template <int States>
    void discretizeA(const Eigen::Matrix<double, States, States> &contA, units::seconds<> dt,
                     Eigen::Matrix<double, States, States> *discA)
    {
        // A_d = eᴬᵀ
        *discA = (contA * dt.value()).exp();
    }

    /**
     * Discretizes the given continuous A and B matrices.
     *
     * @tparam States Number of states.
     * @tparam Inputs Number of inputs.
     * @param contA Continuous system matrix.
     * @param contB Continuous input matrix.
     * @param dt    Discretization timestep.
     * @param discA Storage for discrete system matrix.
     * @param discB Storage for discrete input matrix.
     */
    template <int States, int Inputs>
    void
    discretizeAB(const Eigen::Matrix<double, States, States> &contA,
                 const Eigen::Matrix<double, States, Inputs> &contB, units::seconds<> dt,
                 Eigen::Matrix<double, States, States> *discA, Eigen::Matrix<double, States, Inputs> *discB)
    {
        // M = [A  B]
        //     [0  0]
        Eigen::Matrix<double, States + Inputs, States + Inputs> M;
        M.template block<States, States>(0, 0) = contA;
        M.template block<States, Inputs>(0, States) = contB;
        M.template block<Inputs, States + Inputs>(States, 0).setZero();

        // ϕ = eᴹᵀ = [A_d  B_d]
        //           [ 0    I ]
        auto phi = (M * dt.value()).exp(); // DONT FORGET TO USE MATRIX EXPONENTIAL AFTER REIMPORTING EIGEN

        *discA = phi.template block<States, States>(0, 0);
        *discB = phi.template block<States, Inputs>(0, States);
    }
}