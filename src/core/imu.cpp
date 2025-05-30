#include "api/core/imu.hpp"
#include "api/units.h"

namespace aekulib
{
    IMU::IMU(std::uint8_t port) : imu_(port) {}

    void IMU::reset() { imu_.reset(); }

    bool IMU::isCalibrating() const { return imu_.is_calibrating(); }

    int IMU::tare() { return imu_.tare(); }

    int IMU::tareHeading() { return imu_.tare_heading(); }

    int IMU::tareRotation() { return imu_.tare_rotation(); }

    int IMU::tarePitch() { return imu_.tare_pitch(); }

    int IMU::tareRoll() { return imu_.tare_roll(); }

    int IMU::tareYaw() { return imu_.tare_yaw(); }

    int IMU::tareEuler() { return imu_.tare_euler(); }

    units::angle::degree_t IMU::getHeading() const
    {
        return units::angle::degree_t{imu_.get_heading()};
    }

    void IMU::setHeading(units::angle::degree_t target)
    {
        imu_.set_heading(static_cast<std::uint32_t>(target.to<double>()));
    }

    units::angle::degree_t IMU::getYaw() const
    {
        return units::angle::degree_t{imu_.get_yaw()};
    }

    void IMU::setYaw(units::angle::degree_t target)
    {
        imu_.set_yaw(target.to<double>());
    }

    units::angle::degree_t IMU::getPitch() const
    {
        return units::angle::degree_t{imu_.get_pitch()};
    }

    void IMU::setPitch(units::angle::degree_t target)
    {
        imu_.set_pitch(target.to<double>());
    }

    units::angle::degree_t IMU::getRoll() const
    {
        return units::angle::degree_t{imu_.get_roll()};
    }

    void IMU::setRoll(units::angle::degree_t target)
    {
        imu_.set_roll(target.to<double>());
    }

    IMU::Euler IMU::getEuler() const
    {
        auto e = imu_.get_euler();
        return {units::angle::degree_t{e.roll},
                units::angle::degree_t{e.pitch},
                units::angle::degree_t{e.yaw}};
    }

    void IMU::setEuler(const Euler &target)
    {
        pros::euler_s_t e{target.roll.to<double>(), target.pitch.to<double>(),
                          target.yaw.to<double>()};
        imu_.set_euler(e);
    }

    units::angle::degree_t IMU::getRotation() const
    {
        return units::angle::degree_t{imu_.get_rotation()};
    }

    IMU::Gyro IMU::getGyroRate() const
    {
        auto g = imu_.get_gyro_rate();
        return {units::angular_velocity::degrees_per_second_t{g.x},
                units::angular_velocity::degrees_per_second_t{g.y},
                units::angular_velocity::degrees_per_second_t{g.z}};
    }

    IMU::Accel IMU::getAccel() const
    {
        auto a = imu_.get_accel();
        return {
          units::acceleration::meters_per_second_squared_t{a.x * 9.80665},
          units::acceleration::meters_per_second_squared_t{a.y * 9.80665},
          units::acceleration::meters_per_second_squared_t{a.z * 9.80665}};
    }

    Eigen::Quaterniond IMU::getQuaternion() const
    {
        auto q = imu_.get_quaternion();
        return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
    }
} // namespace aekulib::core
