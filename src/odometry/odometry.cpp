#include "api/odometry/odometry.hpp"
#include "api/chassis/chassisConfiguration.hpp"
#include "api/chassis/kinematics/chassisKinematics.hpp"
#include "api/math/geometry/pose2D.hpp"

namespace aekulib
{
    Odometry::Odometry(const std::shared_ptr<ChassisSensors> &isensors,
                       const std::shared_ptr<ChassisKinematics> &ikinematics)
        : sensors(isensors), kinematics(ikinematics)
    {}

    Pose2D Odometry::update()
    {
        auto angle = sensors->getOrientation() + gyroOffset;
        auto twist = kinematics->toTwist2D(sensors->getEncoderVals(), lastSensorVals);
        twist.dtheta = (angle - lastOrientation).radians();

        auto newPose = pose.exp(twist);

        lastOrientation = angle;
        pose = Pose2D(newPose.getTranslation(), angle);
        lastSensorVals = sensors->getEncoderVals();
        std::cout << "x: " << pose.getX() << ", y:" << pose.getY()
                  << ", theta: " << pose.getRotation().degrees() << '\n';
        return pose;
    }
}