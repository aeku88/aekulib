#include "api/chassis/control/chassisController.hpp"
#include "api/chassis/control/LTVUnicycleController.hpp"
#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/chassis/model/DDChassisModelIntegrated.hpp"
#include "api/math/geometry/pose2D.hpp"
#include "units.h"
#include "units/acceleration.h"
#include "units/velocity.h"
#include <memory>

namespace aekulib
{
    ChassisController::ChassisController(const std::shared_ptr<LTVUnicycleController> &icontroller,
                                         const std::shared_ptr<ChassisConfiguration> &iconfig,
                                         const std::shared_ptr<ChassisSensors> &isensors,
                                         const std::shared_ptr<Odometry> &iodometry,
                                         const std::shared_ptr<MotorGroup> &ileft,
                                         const std::shared_ptr<MotorGroup> &iright)
        : controller(icontroller), config(iconfig), sensors(isensors),
          model(std::make_shared<aekulib::DifferentialDriveChassisModelIntegrated>(ileft, iright)),
          kinematics(std::make_shared<ChassisKinematics>(iconfig)), odometry(iodometry),
          generator({convert<meters_per_second<>>(config->getMaxVelocities().first).value(),
                     convert<meters_per_second_squared<>>(config->getMaxAcceleration()).value(),
                     convert<meters_per_second_squared<>>(config->getMaxAcceleration()).value()})
    {
        controller->setTolerance({1.5_in, 1.5_in, .0349066_rad});
    }

    void ChassisController::setTarget(const std::vector<Pose2D> &iwaypoints, const bool ireverse)
    {
        reverse = ireverse;
        std::vector<squiggles::Pose> waypoints = {};
        if(!reverse)
        {
            for(auto point : iwaypoints)
            {
                waypoints.push_back({convert<meters<>>(point.getX()).value(),
                                     convert<meters<>>(point.getY()).value(),
                                     point.getRotation().radians().value()});
                if(waypoints.size() == 2)
                {
                    auto segment = generator.generate(waypoints);
                    path.insert(path.end(), segment.begin(), segment.end());
                    waypoints.erase(waypoints.begin(), waypoints.end() - 1);
                }
            }
        }

        else
        {
            for(auto it = iwaypoints.rbegin(); it != iwaypoints.rend(); ++it)
            {
                waypoints.push_back({convert<meters<>>((*it).getX()).value(),
                                     convert<meters<>>((*it).getY()).value(),
                                     (*it).getRotation().radians().value()});
                // if(waypoints.size() == 2)
                // {
                //     auto segment = generator.generate(waypoints);
                //     path.insert(path.end(), segment.begin(), segment.end());
                //     waypoints.erase(waypoints.begin(), waypoints.end() - 1);
                // }
            }

            path = generator.generate(waypoints);
        }
    }

    void ChassisController::drive()
    {
        for(std::size_t i = 0; i < path.size(); ++i)
        {
            std::cout << !controller->atReference() << '\n';
            refPose = {path[i].vector.pose.x * m, path[i].vector.pose.y * m, path[i].vector.pose.yaw * rad};
            path[i].curvature /= 39.3701;
            auto chassisVel = controller->calculate(refPose, odometry->update(), path[i].vector.vel * mps,
                                                    path[i].vector.vel * path[i].curvature * rps);
            if(reverse)
                chassisVel = controller->calculate(refPose, odometry->update(), -path[i].vector.vel * mps,
                                                   -path[i].vector.vel * path[i].curvature * rps);
            model->drive(kinematics->inverse(chassisVel));

            pros::delay(10);
        }

        model->drive({0_rpm, 0_rpm});
    }
}