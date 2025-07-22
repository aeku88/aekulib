#include "api/chassis/model/DDChassisModelIntegrated.hpp"

namespace aekulib
{
    DifferentialDriveChassisModelIntegrated::DifferentialDriveChassisModelIntegrated(
      std::shared_ptr<MotorGroup> ileft, std::shared_ptr<MotorGroup> iright)
        : left(ileft), right(iright)
    {}

    void DifferentialDriveChassisModelIntegrated::step() {}

    void DifferentialDriveChassisModelIntegrated::drive(
      const Eigen::Vector2<revolutions_per_minute<>> targetVelocities)
    {
        left->move(targetVelocities[0]);
        right->move(targetVelocities[1]);
    }
}