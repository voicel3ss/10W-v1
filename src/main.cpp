#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup left_motor_group({11, -12}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({-19, 20}, pros::MotorGears::blue);

pros::MotorGroup lift({1, -9}, pros::MotorGears::blue);

pros::MotorGroup claw_rollers({6, -7}, pros::MotorGears::blue);

pros::Motor claw(8, pros::MotorGears::blue);

pros::Rotation lift_encoder(10);

lemlib::Drivetrain drivetrain(&left_motor_group,  // left motor group
							  &right_motor_group, // right motor group
							  10,				  // track width
							  2.75,				  // wheel size
							  360,				  // drivetrain rpm
							  2					  // horizontal drift
);

pros::Imu imu(2);
pros::Rotation horizontal_encoder(5);
pros::Rotation vertical_encoder(21);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2.75, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2.75, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel,	// vertical tracking wheel 1
							nullptr,					// vertical tracking wheel 2
							&horizontal_tracking_wheel, // horizontal tracking wheel 1
							nullptr,					// horizontal tracking wheel 2
							&imu						// inertial sensor
);

lemlib::ControllerSettings lateral_controller(10,  // proportional gain (kP)
											  0,   // integral gain (kI)
											  3,   // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in inches
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in inches
											  500, // large error range timeout, in milliseconds
											  20   // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
											  0,   // integral gain (kI)
											  10,  // derivative gain (kD)
											  3,   // anti windup
											  1,   // small error range, in degrees
											  100, // small error range timeout, in milliseconds
											  3,   // large error range, in degrees
											  500, // large error range timeout, in milliseconds
											  0	   // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain,			// drivetrain settings
						lateral_controller, // lateral PID settings
						angular_controller, // angular PID settings
						sensors				// odometry sensors
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize()
{
	pros::lcd::initialize();
	chassis.calibrate();
	lift_encoder.reset_position();

	static pros::Task screen_task([&]()
								  {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
			pros::lcd::print(3, "Lift: %.2f", lift_encoder.get_position() / 100.0);
            pros::delay(100);
        } });
}

void mechanism_control()
{
	while (true)
	{
		// Lift
		double lift_position = lift_encoder.get_position() / 100.0;

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
		{
			lift.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
		{
			double x = std::clamp(lift_position / 1200.0, 0.0, 1.0);
			double curve = 3 * x * x - 2 * x * x * x;

			/*
			curve is currrently squared.
			cube it to make the lift more cautious,
			or make it linear to make it more aggressive
			*/
			int speed = 20 + 107 * curve * curve;

			lift.move(lift_position <= 5 ? 0 : -speed);
		}
		else
		{
			lift.move(0);
		}

		// Rollers
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			claw_rollers.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			claw_rollers.move(-127);
		}
		else
		{
			claw_rollers.move(0);
		}

		// Claw
		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) ||
			controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
		{
			claw.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) ||
				 controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
		{
			claw.move(-127);
		}
		else
		{
			claw.move(0);
		}

		pros::delay(20);
	}
}

void opcontrol()
{
	lift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	claw_rollers.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

	pros::Task mechanism_task(mechanism_control);

	while (true)
	{
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(leftY, rightX);

		pros::delay(25);
	}
}