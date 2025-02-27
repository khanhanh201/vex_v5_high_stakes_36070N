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
#define ladybrown_port 6

//IMU
#define IMU_port 7

//Encoder
#define horizontal_encoder_port 15 //tbd
#define vertical_encoder_port 16//tbd
#define ladybrown_encoder_port 5

//Pistons
#define mogo_pneumatic_port 'C'
#define doinker_pneumatic_port 'A'

//Ladybrown
#define ladybrown_deadband 2

//Time
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

//CONTROLLER
pros::Controller master(pros::E_CONTROLLER_MASTER);

//MOTOR GROUPS
pros::MotorGroup left_motor_group({left_motor_port_1, left_motor_port_2, left_motor_port_3}, pros::MotorGearset::green);
pros::MotorGroup right_motor_group({right_motor_port_1, right_motor_port_2, right_motor_port_3}, pros::MotorGearset::green);
pros::MotorGroup intake({intake_port_1, intake_port_2}, pros::MotorGearset::green);
pros::Motor ladybrown(ladybrown_port, pros::MotorGearset::green);

//IMU AND ROTATION SENSORS
pros::IMU imu(IMU_port);
pros::Rotation ladybrown_encoder(ladybrown_encoder_port);
pros::Rotation horizontal_encoder(horizontal_encoder_port);
pros::Rotation vertical_encoder(vertical_encoder_port);

//PNEUMATICS
pros::ADIDigitalOut mogo_pneumatic(mogo_pneumatic_port);
pros::ADIDigitalOut doinker_pneumatic(doinker_pneumatic_port);





//Tracking wheels, wheel type and offset to be determined
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -2.56);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 2.36); //phai tune lai

//Drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, 
							  &right_motor_group,
							  12.3, //track width
							  lemlib::Omniwheel::NEW_4, //wheel type
							  280, //rpm
							  8 //horizontal drift
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
                                            3, // anti-windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            100 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, //. proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             2, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

//Creating the base
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);




//intake--
//mogo--
//doinker--
//vision--
//ladybrown
//hang???


my_custom_PID ladybrown_PID(6, 0, 0.5);
double rotation_error;
int rotation_count;
bool move_done = true;
double ladybrown_target = 0;
bool enable_pid = false;
int ladybrown_time_count = 10;

void ladybrown_target_set(double target)
{
    ladybrown_target = target;
    enable_pid = true;
}

void ladybrown_move_PID()
{
    pros::lcd::set_text(4, "not done");
	move_done = false;
    rotation_count = 0;
	ladybrown_PID.reset();

	while (!move_done)
	{
		rotation_error = ladybrown_target - ((double)(ladybrown_encoder.get_position())/100);
		ladybrown.move_velocity(ladybrown_PID.update(rotation_error));

	    if (std::abs(rotation_error) < ladybrown_deadband) rotation_count++;
        if (rotation_count >= ladybrown_time_count) move_done = true;

		pros::delay(delay_time);
	}
    pros::lcd::set_text(4, "done");
    ladybrown.brake();
    ladybrown_PID.reset();
    enable_pid = false;
}

void ladybrown_task()
{
    while (true)
    {
        if (enable_pid)
        {
            ladybrown_move_PID();
            enable_pid = false;
            ladybrown_PID.reset();
        }
        pros::delay(delay_time);
    }
}

pros::Task Ladybrown_Task(ladybrown_task);



void intake_move(int speed)
{
    intake.move_velocity(speed);
}

void mogo_move(bool x)
{
    mogo_pneumatic.set_value(!x);
}

void doinker_move(bool x)
{
    doinker_pneumatic.set_value(x);
}





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

    left_motor_group.tare_position();
    right_motor_group.tare_position();
    intake.tare_position();
    ladybrown.tare_position();
    horizontal_encoder.reset_position();
    ladybrown_encoder.reset_position();
    ladybrown_encoder.set_reversed(true);
    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

            // log telemetry sink
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(delay_time);
        }
    });

    ladybrown.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() {}

ASSET(final_path_1_txt);
ASSET(final_path_2_txt);
ASSET(final_path_3_txt);

void autonomous()
{
    //start
    chassis.setPose(-60, 0, 90);
    mogo_move(false);

    //score the alliance wall stake
    intake_move(200);
    pros::delay(600);

    //move forward and turn 90
    chassis.moveToPoint(-47, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 1000);

    //move to the mogo and grasp
    chassis.moveToPoint(-46.5, -16.5, 2000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(300);

    //follow path and score rings
    chassis.turnToHeading(113, 1000);
    chassis.follow(final_path_1_txt, 20, 5000);
    chassis.waitUntilDone();
    pros::delay(200);
    
    //move to the alliance stake
    chassis.moveToPoint(3.75, -45, 800, {.forwards = false});
    chassis.waitUntilDone();
    ladybrown_target_set(26);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(4, -61, 2000);
    chassis.waitUntilDone();
    pros::delay(600);

    //score ladybrown
    intake_move(0);
    intake.brake();
    ladybrown_target_set(130);
    pros::delay(1000);
    ladybrown_target_set(0);
    ladybrown.brake();
    chassis.setPose(0, -58, 180);

    //take in 3 rings in a row
    chassis.moveToPoint(0, -40, 1000, {.forwards = false});
    intake_move(200);
    chassis.waitUntilDone();
    chassis.moveToPoint(-62, -46.5, 10000, {.maxSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(400);

    //take in the final ring
    chassis.moveToPose(-44, -64, 135, 1200);
    chassis.waitUntilDone();
    pros::delay(500);

    //move to the corner
    chassis.moveToPose(-62, -72, -45, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(-100);
    mogo_move(false);
    pros::delay(200);
    intake_move(0);

    //move to the center
    chassis.moveToPoint(0, 0, 20000);
    chassis.waitUntil(76);
    intake_move(200);
    chassis.waitUntilDone();
    intake_move(0);
    chassis.moveToPoint(-26, -26, 5000);
    chassis.waitUntilDone();
    mogo_move(true);

    //grab the mogo
    chassis.follow(final_path_3_txt, 20, 20000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-48.7, 12.6, 10000, {.forwards = false, .maxSpeed = 70});
}


void opcontrol() {
    while (true)
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            chassis.turnToHeading(90, 10000);
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            chassis.turnToHeading(0, 10000);
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 24, 10000);
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -24, 10000);
            master.rumble(".");
        }

        pros::delay(delay_time);
    }
}