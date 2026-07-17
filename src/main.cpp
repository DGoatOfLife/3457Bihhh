#include "main.h"
#include "lemlib/api.hpp"

using namespace lemlib;

MotorGroup left_motors({-10, -11}, MotorGearset::blue);
MotorGroup right_motors({3, 20}, MotorGearset::blue);
MotorGroup Scoring({-21, 19}, MotorGearset::blue);
Controller controller(E_CONTROLLER_MASTER);

// track width: 11.5"
// wheel dia: 3.25"
// horiz drift: 2
// track length: 14.5"

Drivetrain drivetrain(
	&left_motors,
	&right_motors,
	11.5,
	Omniwheel::NEW_325,
	450,
	2
);

Imu imu(18);

OdomSensors sensors(
	nullptr,	// vert track wheel 1
	nullptr,	// vert track wheel 2
	nullptr,	// horiz track wheel 1
	nullptr,	// horiz track wheel 2
	&imu		// IMU
);

ControllerSettings lateral_controller(
	10,		// kP
	0,		// kI
	3,		// kD
	3,		// anti windup
	1,		// small error range (in)
	100,	// small error range timeout (ms)
	3,		// large error range (in)
	500,	// large error range timeout (ms)
	20		// max accel (slew)
);

// kP 2 kD 10
ControllerSettings angular_controller(
	3,		// kP
	0,		// kI
	13,		// kD
	0,		// anti windup
	0,		// small error range (in)
	0,	// small error range timeout (ms)
	0,		// large error range (in)
	0,	// large error range timeout (ms)
	0		// max accel (slew)
);

Chassis chassis(
	drivetrain,
	lateral_controller,
	angular_controller,
	sensors
);

// SUBSYSTEM INITIALIZATION
Lift lift = Lift();


// motor leftMotorA = motor(PORT10, ratio6_1, true);
// motor leftMotorB = motor(PORT11, ratio6_1, true);
// motor rightMotorA = motor(PORT3, ratio6_1, true);
// motor rightMotorB = motor(PORT20, ratio6_1, false);

// motor ScoringMotorA = motor(PORT21, ratio6_1, true);
// motor ScoringMotorB = motor(PORT19, ratio6_1, false);
// motor_group Scoring = motor_group(ScoringMotorA, ScoringMotorB);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	chassis.calibrate();
	Lift::resetPosition();
	Lift::motors.set_brake_mode_all(E_MOTOR_BRAKE_HOLD);

	Task screen_task([&]() {
		while (true) {
			// print odom position to brain screen
			pros::lcd::print(0, "X: %f", chassis.getPose().x);
			pros::lcd::print(1, "Y: %f", chassis.getPose().y);
			pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			// delay to save resources
			pros::delay(20);
		}
	});

	Task controller_task([&]() {
		while (true) {
			controller.clear_line(0);
			controller.print(2, 0, "kP: %d | kD: %d", angular_controller.kP, angular_controller.kD);
			pros::delay(100);
		}
	});
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
void autonomous() {
	angularPIDTest();
}

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
void opcontrol() {
	while (true) {
		// Adjust lift targets.
		if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			Lift::setLiftTarget(Lift::A);
		} else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_B)) {
			Lift::setLiftTarget(Lift::B);
		} else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
			Lift::setLiftTarget(Lift::C);
		} else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
			Lift::setLiftTarget(Lift::D);
		} else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)) {
			Lift::setLiftTarget(Lift::E);
		} else if (controller.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
			Lift::setLiftTarget(Lift::F);
		}

		Lift::handleControllerInput(
			controller.get_digital(E_CONTROLLER_DIGITAL_L2),
			controller.get_digital(E_CONTROLLER_DIGITAL_L1)
		);

		if (controller.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			Scoring.move(127);
		} else if (controller.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			Scoring.move(-127);
		} else {
			Scoring.move(-11);
		}


		// Drivetrain control.
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		chassis.tank(leftY, rightY);

		// delay to save resources
		pros::delay(20);
	}
}