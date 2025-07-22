#pragma once

#include "api/chassis/model/DDChassisModel.hpp"
#include "api/devices/motorGroup.hpp"
#include <memory>

namespace aekulib
{
    class DifferentialDriveChassisModelIntegrated : public DifferentialDriveChassisModel
    {
      public:
        DifferentialDriveChassisModelIntegrated(std::shared_ptr<MotorGroup> ileft,
                                                std::shared_ptr<MotorGroup> iright);

        void step() override;

        void drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities) override;

      private:
        std::shared_ptr<MotorGroup> left, right;
    };
}