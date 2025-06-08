#include "api/chassis/model/DDChassisModelPID.hpp"
#include "pros/rtos.hpp"
#include "units/impedance.h"

namespace aekulib
{
    DifferentialDriveChassisModelPID::DifferentialDriveChassisModelPID(
      const std::shared_ptr<MotorGroup> ileft, const PIDController<revolutions_per_minute<>> ileftController,
      const std::shared_ptr<MotorGroup> iright,
      const PIDController<revolutions_per_minute<>> irightController)
        : DifferentialDriveChassisModel(ileft, iright), leftController(ileftController),
          rightController(irightController)
    {
    }

    void
    DifferentialDriveChassisModelPID::drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities)
    {
        leftController.setTarget(targetVelocities[0]);
        rightController.setTarget(targetVelocities[1]);
    }

    void DifferentialDriveChassisModelPID::step()
    {
        left->move(leftController.getOutput());
        right->move(rightController.getOutput());
        pros::delay(10);
    }
}