//TUNING PID: TUNING P GAIN AND D GAIN THROUGH ANGULAR MOVEMENT

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"

//Motors
#define left_motor_port_1 -1
#define left_motor_port_2 -2
#define left_motor_port_3 3
#define right_motor_port_1 -8
#define right_motor_port_2 9
#define right_motor_port_3 10
#define intake_port_1 -12
#define intake_port_2 11
#define ladybrown_port -6

//IMU
#define IMU_port 7

//Encoder
#define horizontal_encoder_port 1 //tbd
#define vertical_encoder_port 1 //tbd
#define ladybrown_encoder_port 5

//Pistons
#define mogo_pneumatic_port 'C'
#define doinker_pneumatic_port 'A'

//Vision
#define vision_port 1 //tbd

//Speed
#define intake_conveyor_speed 200 
#define ladybrown_speed 150 //tbd

//Time
//PROS run tasks every 2 millis
#define delay_time 10


/**
 * To be determined:
 * Ports
 * Whether to reverse motors
 * Whether to reverse sensors
 * Pneumatics initial states
 * Wheels and gears (wheel diameter, gear ratio, tracking wheels offset)
 * Tuning
 */

//Controller
pros::Controller master(pros::E_CONTROLLER_MASTER);

//Motor groups
pros::MotorGroup left_motor_group({left_motor_port_1, left_motor_port_2, left_motor_port_3}, pros::MotorGearset::green);
pros::MotorGroup right_motor_group({right_motor_port_1, right_motor_port_2, right_motor_port_3}, pros::MotorGearset::green);
pros::MotorGroup intake({intake_port_1, intake_port_2}, pros::MotorGearset::green);
pros::Motor ladybrown(ladybrown_port, pros::MotorGearset::green);

//IMU and rotation sensors
pros::IMU imu(IMU_port);
pros::Rotation ladybrown_encoder(ladybrown_encoder_port);
pros::Rotation horizontal_encoder(horizontal_encoder_port);
pros::Rotation vertical_encoder(vertical_encoder_port);

//Pneumatic
pros::ADIDigitalOut mogo_pneumatic(mogo_pneumatic_port);
pros::ADIDigitalOut doinker_pneumatic(doinker_pneumatic_port);

//Vision sensor
pros::Vision vision_sensor(vision_port);





//Tracking wheels, wheel type and offset to be determined
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 1);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 1);

//Drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, 
							  &right_motor_group,
							  10, //track width
							  lemlib::Omniwheel::NEW_4, //wheel type
							  280, //rpm
							  2 //horizontal drift
);

//Sensors for odometry
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal_tracking_wheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

//Two PID controllers: lateral and angular
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            0, // anti-windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//Creating the base
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            pros::lcd::print(3, "Horizontal encoder: %i", horizontal_encoder.get_position());
            pros::lcd::print(4, "Vertical encoder: %i", vertical_encoder.get_position());

            pros::lcd::print(5, "Horizontal encoder distance traveled: %f", horizontal_tracking_wheel.getDistanceTraveled());
            pros::lcd::print(6, "Vertical encoder distance traveled: %f", vertical_tracking_wheel.getDistanceTraveled());

            // log telemetry sink
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(delay_time);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous()
{
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(90, 100000);
}

void opcontrol() {
    //run autonomous code
    autonomous();
}