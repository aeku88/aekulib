#include "main.h"
#include "pros/misc.h"
#include "units/velocity.h"

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

std::vector<int8_t> leftPorts = {1, 11}, rightPorts = {-10, -18};

auto left = std::make_shared<aekulib::MotorGroup>(leftPorts),
     right = std::make_shared<aekulib::MotorGroup>(rightPorts);

// SECOND PARAM IS TRACK WIDTH PLS REMEASURE AND CHANGE ITS WRONG RN
auto config
  = std::make_shared<aekulib::ChassisConfiguration>(2.75_in, 10_in, pros::MotorGears::blue, 1.0, 12_lb);

auto kinematics = aekulib::ChassisKinematics(config);

auto controllerGains = aekulib::PIDGains(.25, 0.0, 0.0);

auto leftController = aekulib::PIDController<revolutions_per_minute<>>(controllerGains),
     rightController = aekulib::PIDController<revolutions_per_minute<>>(controllerGains);

auto model = aekulib::DifferentialDriveChassisModelPID(left, leftController, right, rightController);

void opcontrol()
{
    /*pros::Controller master(pros::E_CONTROLLER_MASTER);

    while(true)
    {
        inches_per_second<> linVel
          = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0 * 86.39 * ips;
        radians_per_second<> angVel
          = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 * 17.278 * rps;

        model.drive(kinematics.inverse({linVel, angVel}));

        pros::delay(10);
    }*/

    std::ofstream outputFile("data.csv");

    aekulib::motorKt();
}