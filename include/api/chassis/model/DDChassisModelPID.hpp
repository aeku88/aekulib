#include "api/chassis/model/DDChassisModel.hpp"
#include "api/control/PIDController.hpp"
#include "pros/rtos.hpp"

using namespace units::literals;

namespace aekulib
{
    class DifferentialDriveChassisModelPID : public DifferentialDriveChassisModel
    {
      public:
        DifferentialDriveChassisModelPID(const std::shared_ptr<MotorGroup> ileft,
                                         const PIDController<revolutions_per_minute<>> ileftController,
                                         const std::shared_ptr<MotorGroup> iright,
                                         const PIDController<revolutions_per_minute<>> irightController);
        void drive(const Eigen::Vector2<revolutions_per_minute<>> targetVelocities) override;

        void step() override;

      private:
        std::shared_ptr<MotorGroup> left, right;
        void loop();

        pros::Task loopTask;

        PIDController<revolutions_per_minute<>> leftController, rightController;
    };
}