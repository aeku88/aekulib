#include "api/chassis/model/DDChassisModelIntegrated.hpp"
#include "pros/abstract_motor.hpp"

namespace aekulib
{
    DifferentialDriveChassisModelIntegrated::DifferentialDriveChassisModelIntegrated(
      std::shared_ptr<MotorGroup> ileft, std::shared_ptr<MotorGroup> iright)
        : left(ileft), right(iright)
    {
        left->setBrakeMode(pros::v5::MotorBrake::brake);
        right->setBrakeMode(pros::v5::MotorBrake::brake);
    }

    void DifferentialDriveChassisModelIntegrated::step() {}

    void DifferentialDriveChassisModelIntegrated::drive(
      const Eigen::Vector2<revolutions_per_minute<>> targetVelocities)
    {
        left->move(targetVelocities[0]);
        right->move(targetVelocities[1]);
    }
}