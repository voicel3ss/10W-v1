#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup left_motor_group({-1, 2, -3}, pros::MotorGears::blue);
pros::MotorGroup right_motor_group({4, -5, 6}, pros::MotorGears::green);

pros::MotorGroup lift({7, 8}, pros::MotorGears::green);

pros::Motor claw(9, pros::MotorGears::green);

lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10, // track width
                              2.75, // wheel size
                              360, // drivetrain rpm
                              2 // horizontal drift
);

pros::Imu imu(10);
pros::Rotation horizontal_encoder(20);
pros::Rotation vertical_encoder(21);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, 2.75, 0);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, 2.75, 0);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

	pros::Task screen_task([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(20);
        }
    });
}

void opcontrol() {
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        chassis.arcade(leftY, leftX);
        pros::delay(25);
    }
}