#include "api/devices/imu.hpp"

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

    degrees<> IMU::getHeading() const { return degrees(imu_.get_heading()); }

    void IMU::setHeading(degrees<> target) { imu_.set_heading(target.to<int32_t>()); }

    degrees<> IMU::getYaw() const { return degrees(imu_.get_yaw()); }

    void IMU::setYaw(degrees<> target) { imu_.set_yaw(target.to<int32_t>()); }

    degrees<> IMU::getPitch() const { return degrees<>(imu_.get_pitch()); }

    void IMU::setPitch(degrees<> target) { imu_.set_pitch(target.to<int32_t>()); }

    degrees<> IMU::getRoll() const { return degrees(imu_.get_roll()); }

    void IMU::setRoll(degrees<> target) { imu_.set_roll(target.to<int32_t>()); }

    degrees<> IMU::getRotation() const { return degrees(imu_.get_rotation()); }

    Eigen::Vector3<inches_per_second_squared<>> IMU::getAccel() const
    {
        auto a = imu_.get_accel();
        return {meters_per_second_squared(a.x * 9.80665), meters_per_second_squared(a.y * 9.80665),
                meters_per_second_squared(a.z * 9.80665)};
    }
} // namespace aekulib::core
