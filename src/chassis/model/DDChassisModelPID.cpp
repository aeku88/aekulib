#include "api/chassis/model/DDChassisModelPID.hpp"

namespace aekulib
{
    DifferentialDriveChassisModelPID::DifferentialDriveChassisModelPID(
      const std::shared_ptr<MotorGroup> ileft, const PIDController<revolutions_per_minute<>> ileftController,
      const std::shared_ptr<MotorGroup> iright,
      const PIDController<revolutions_per_minute<>> irightController)
        : left(ileft), right(iright), leftController(ileftController), rightController(irightController),
          loopTask([this] { this->loop(); })
    {}

    void DifferentialDriveChassisModelPID::loop()
    {
        while(true)
        {
            step();
            pros::delay(10);
        }
    }

    void
    DifferentialDriveChassisModelPID::drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities)
    {
        leftController.setTarget(targetVelocities[0]);
        rightController.setTarget(targetVelocities[1]);
    }

    void DifferentialDriveChassisModelPID::step()
    {
        leftController.step();
        rightController.step();

        left->move(leftController.getOutput());
        right->move(rightController.getOutput());
    }
}