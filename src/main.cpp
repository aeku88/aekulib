#include "main.h"
#include "Eigen/src/Core/Matrix.h"
#include "api/devices/rotationSensor.hpp"
#include "api/math/geometry/pose2D.hpp"
#include "api/odometry/odometry.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "squiggles/math/utils.hpp"
#include "units/core.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button()
{
    static bool pressed = false;
    pressed = !pressed;
    if(pressed)
        pros::lcd::set_text(2, "I was pressed!");
    else
        pros::lcd::clear_line(2);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Hello PROS User!");

    pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

std::vector<int8_t> leftPorts = {-11, -12, -13}, rightPorts = {18, 19, 20};

auto left = std::make_shared<aekulib::MotorGroup>(leftPorts, pros::MotorGears::blue),
     right = std::make_shared<aekulib::MotorGroup>(rightPorts, pros::MotorGears::blue);

auto leftRot = std::make_shared<aekulib::RotationSensor>(1),
     rightRot = std::make_shared<aekulib::RotationSensor>(-10);

auto rollers = std::make_shared<aekulib::Motor>(1), topRoller = std::make_shared<aekulib::Motor>(2);

auto imu = std::make_shared<aekulib::IMU>(9);

auto model = std::make_shared<aekulib::DifferentialDriveChassisModelIntegrated>(left, right);

auto config = std::make_shared<aekulib::ChassisConfiguration>(3.25_in, 8.375_in, pros::MotorGears::blue,
                                                              36.0 / 48.0, 6_lb);

auto kinematics = std::make_shared<aekulib::ChassisKinematics>(config);

auto sensors = std::make_shared<aekulib::ChassisSensors>(leftRot, rightRot, imu, config);

auto odometry = std::make_shared<aekulib::Odometry>(sensors, kinematics);

auto controller = LTVUnicycleController({0.1, 0.1, 2.0}, {1.0, 2.0}, 10_ms, config->getMaxVelocities().first);

const double MAX_VEL = convert<meters_per_second<>>(config->getMaxVelocities().first).value();
const double MAX_ACCEL = convert<meters_per_second_squared<>>(config->getMaxAcceleration()).value();
const double MAX_JERK = MAX_ACCEL * 1.5;

pros::ADIDigitalOut indexer = pros::adi::DigitalOut('G', HIGH);
bool state_indexer = true;

auto aligner = pros::adi::DigitalOut('H', LOW);
bool state_aligner = true;

void opcontrol()
{
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    /*while(true)
    {
        inches_per_second<> linVel
          = squiggles::sgn(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0)
            * std::pow(fabs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0), 3)
            * config->getMaxVelocities().first;
        radians_per_second<> angVel
          = squiggles::sgn(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0)
            * std::pow(fabs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0), 1.1)
            * config->getMaxVelocities().second * .75;

        model->drive(kinematics->inverse({linVel, angVel}));

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
            rollers->move(600_rpm);
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
            rollers->move(-600_rpm);
        else
            rollers->move(0_rpm);

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
            topRoller->move(600_rpm);
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
            topRoller->move(-600_rpm);
        else
            topRoller->move(0_rpm);

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
        {
            state_indexer = !state_indexer;
            indexer.set_value(state_indexer);
        }

        else if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            state_aligner = !state_aligner;
            aligner.set_value(state_aligner);
        }

        pros::delay(10);
    }*/

    imu->reset();
    while(imu->isCalibrating())
        pros::delay(5);
    imu->tare();

    auto constraints = squiggles::Constraints(MAX_VEL, MAX_ACCEL, MAX_JERK);
    auto generator = squiggles::SplineGenerator(
      constraints,
      std::make_shared<squiggles::TankModel>(convert<meters<>>(config->getTrackWidth()).value(), constraints),
      .01);

    auto path = generator.generate({squiggles::Pose(0.0, 0.0, 0.0), squiggles::Pose(.3, .3, 0)});
    for(std::size_t i = 0; i < path.size(); ++i)
    {
        auto right = convert<inches_per_second<>>(path[i].wheel_velocities[0] * mps)
                     * (360_deg / (config->getWheelDiameter() * pi)) * config->getGearRatio();
        auto left = convert<inches_per_second<>>(path[i].wheel_velocities[1] * mps)
                    * (360_deg / (config->getWheelDiameter() * pi)) * config->getGearRatio();
        Pose2D refPose
          = {path[i].vector.pose.x * m, path[i].vector.pose.y * m, path[i].vector.pose.yaw * rad};
        path[i].curvature /= 39.3701;
        // std::cout << path[i].vector.vel * mps << ", " << path[i].vector.vel * path[i].curvature * rps <<
        // '\n';
        auto chassisVel = controller.calculate(refPose, odometry->update(), path[i].vector.vel * mps,
                                               path[i].vector.vel * path[i].curvature * rps);

        model->drive(kinematics->inverse(path[i].vector.vel * mps, path[i].curvature));
        pros::delay(10);
    }
    model->drive({0_rpm, 0_rpm});
}