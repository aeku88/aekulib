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
        auto twist = kinematics->toTwist2D(sensors->getEncoderVals(), lastSensorVals);
        twist.dtheta = (sensors->getOrientation() - lastOrientation).radians();

        auto newPose = pose.exp(twist);

        lastOrientation = sensors->getOrientation();
        pose = Pose2D(newPose.getTranslation(), sensors->getOrientation());
        lastSensorVals = sensors->getEncoderVals();
        return pose;
    }
}