#include "main.h"
#include "liblvgl/core/lv_disp.h" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_pos.h" // IWYU pragma: keep
#include "liblvgl/widgets/lv_img.h" // IWYU pragma: keep
#include "stormlib/led.hpp" // IWYU pragma: keep
#include "stormlib/selector.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "liblvgl/core/lv_obj_class.h" // IWYU pragma: keep
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"   // IWYU pragma: keep
#include "pros/adi.hpp"   // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include <algorithm>	// IWYU pragma: keep
#include <cmath>

stormlib::selector autonSelector(stormlib::selector::E_BLUE_RIGHT_4, "AWP", "Safe Toggle", "Goal Rush", "Safe Goal");

//Cascade heights
double cascadeHeight0 = 0; // bottom height for cascade
double cascadeHeight1 = 200; // low height for cascade
double cascadeHeight2 = 400; // mid height for cascade
double cascadeHeight3 = 600; // high height for cascade
double cascadeHeight4 = 800; // max height for cascade

//Cascade values
double cascadeKp = 0.9; // proportional gain for cascade PID
double cascadeKi = 0.0; // integral gain for cascade PID
double cascadeKd = 0.0; // derivative gain for cascade PID
double cascadePIDTarget = 0.0; // target position for cascade PID
volatile double cascadeTargetValue = 0.0; // target position for cascade PID

//Alliance colour
bool isRedAlliance = true; // true if red alliance, false if blue alliance

//Claw toggle
bool clawToggle = true; // true if claw is closed, false if claw is open

//Controllers
pros::Controller Maverick(pros::E_CONTROLLER_MASTER);
pros::Controller Goose(pros::E_CONTROLLER_PARTNER);

//claw piston
pros::adi::DigitalOut claw('A');

// left motor group
pros::MotorGroup left_motor_group({-11, -12}, pros::MotorGearset::blue);
// right motor group
pros::MotorGroup right_motor_group({19, 20}, pros::MotorGearset::blue);
// four bar lift motor group
pros::MotorGroup cascade({14, -17}, pros::MotorGearset::blue);

//Left Active Toggle
pros::Motor toggleLeft(15, pros::MotorGearset::green);

//Right Active Toggle
pros::Motor toggleRight(16, pros::MotorGearset::green);

// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              14, // 14 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              366.6667, // drivetrain rpm is 366.6667
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(1);

// horizontal tracking wheel encoder 1
pros::Rotation horizontal_encoder_1(2);

// vertical tracking wheel encoder 1
pros::Rotation vertical_encoder_1(-3);

//Left optical sensor
pros::Optical opticalLeft(9);

//Right optical sensor
pros::Optical opticalRight(10);

// horizontal tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder_1, lemlib::Omniwheel::NEW_2, -5.75);

// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder_1, lemlib::Omniwheel::NEW_2, 2.6);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1
                            nullptr, // vertical tracking wheel 2 
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              5, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.23, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void cascadePIDTask(void* param) {
    // Variables to track error across loops
    double cascadeError = 0;
    double cascadePrevError = 0;
    double cascadeIntegral = 0;
    double cascadeDerivative = 0;
    
    while (true) {
        // Get the current position (average of left and right encoders)
        double cascadeCurrentPosition = cascade.get_position();
        
        // Calculate Proportional (Current Error)
        cascadeError = cascadeTargetValue - cascadeCurrentPosition;
        
        // Calculate Integral (Accumulated Error)
        // Only integrate if we are close to the target to avoid "integral windup"
        if (fabs(cascadeError) < 50) { 
            cascadeIntegral += cascadeError;
        } else {
            cascadeIntegral = 0;
        }
        
        // Calculate Derivative (Rate of Change)
        cascadeDerivative = cascadeError - cascadePrevError;
        cascadePrevError = cascadeError;
        
        // Calculate the total PID output power
        double cascadeOutputPower = (cascadeError * cascadeKp) + (cascadeIntegral * cascadeKi) + (cascadeDerivative * cascadeKd);
        
        // Cap the max/min bounds to match PROS .move() limits
        if (cascadeOutputPower > 127) cascadeOutputPower = 127;
        if (cascadeOutputPower < -127) cascadeOutputPower = -127;
        
        // Command the motor using PROS syntax
        cascade.move(cascadeOutputPower);
        pros::delay(20);   
    }
}

// initialize function. Runs on program startup
void initialize() {
    chassis.calibrate(); // calibrate sensors
    cascade.tare_position_all(); // reset cascade motor positions
	autonSelector.initialize(); // initialize auton selector
    };


void autonomous() {

}

void opcontrol() {
    // pros::Task cascadePID(cascadePIDTask); // start the cascade PID task
    // loop forever
    while (true) {
        // get left y and right x positions
        int leftY = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = Maverick.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        //Cascade controls
        // if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        //     if (clawToggle == true) {
        //         claw.set_value(false); //Open claw
        //         pros::delay(200);
        //         cascadeTargetValue = cascadeHeight0;
        //         clawToggle = !clawToggle;
        //     } else {
        //         claw.set_value(true); //Close claw
        //         cascadeTargetValue = cascadeHeight0 + 50; //Raise cascade slightly to avoid hitting the ground
        //         clawToggle = !clawToggle;
        //     }
        // } //Bottom cascade height
        // else if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
        //     cascadeTargetValue = cascadeHeight1;
        // } //First cascade height
        // else if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
        //     cascadeTargetValue = cascadeHeight2;
        // } //Second cascade height
        // else if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        //     cascadeTargetValue = cascadeHeight3;
        // } //Third cascade height
        // else if (Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
        //     cascadeTargetValue = cascadeHeight4;
        // } //Fourth cascade height
        // else {
        //     cascade.move(0); 
        //     cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        // } //stop cascade and hold position

        if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {cascade.move(127);} //Raise cascade
        else if (Maverick.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {cascade.move(-127);} //Lower cascade
        else {cascade.move(0); cascade.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //stop cascade and hold position

        if (isRedAlliance == true) { // Red alliance controls
            //Left toggle controls
            if (opticalLeft.get_hue() < 10 && opticalLeft.get_hue() > 0) {toggleLeft.move(127);} //Red detected
            else if (opticalLeft.get_hue() > 200 && opticalLeft.get_hue() < 250) {toggleLeft.move(-127);} //Blue detected
            else {toggleLeft.move(0); toggleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //No color detected, stop and hold position

            //Right toggle controls
            if (opticalRight.get_hue() < 10 && opticalRight.get_hue() > 0) {toggleRight.move(127);} //Red detected
            else if (opticalRight.get_hue() > 200 && opticalRight.get_hue() < 250) {toggleRight.move(-127);} //Blue detected
            else {toggleRight.move(0); toggleRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //No color detected, stop and hold position
        }
        else { // Blue alliance controls
            //Left toggle controls
            if (opticalLeft.get_hue() < 10 && opticalLeft.get_hue() > 0) {toggleLeft.move(-127);} //Red detected
            else if (opticalLeft.get_hue() > 200 && opticalLeft.get_hue() < 250) {toggleLeft.move(127);} //Blue detected
            else {toggleLeft.move(0); toggleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //No color detected, stop and hold position

            //Right toggle controls
            if (opticalRight.get_hue() < 10 && opticalRight.get_hue() > 0) {toggleRight.move(-127);} //Red detected
            else if (opticalRight.get_hue() > 200 && opticalRight.get_hue() < 250) {toggleRight.move(127);} //Blue detected
            else {toggleRight.move(0); toggleRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);} //No color detected, stop and hold position
        }

        if(Goose.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {claw.set_value(true);} //Close claw

        // delay to save resources
        pros::delay(25);
}
}
