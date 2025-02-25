/**
 * TUNING PID : TUNING P GAIN AND D GAIN THROUGH ANGULAR MOVEMENT
 * Tracking:
 * - heading: IMU
 * - vertical: IME (base)
 * - horizontal: tracking wheel
*/

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
#define ladybrown_encoder_port 5

//Pistons
#define mogo_pneumatic_port 'C'
#define doinker_pneumatic_port 'A'

//Ladybrown
#define ladybrown_deadband 2

//Speed
#define intake_conveyor_speed 200 

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

//PNEUMATICS
pros::ADIDigitalOut mogo_pneumatic(mogo_pneumatic_port);
pros::ADIDigitalOut doinker_pneumatic(doinker_pneumatic_port);

//VISION SENSOR





//Tracking wheels, wheel type and offset to be determined
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.5);
lemlib::TrackingWheel vertical_tracking_wheel(&right_motor_group, lemlib::Omniwheel::NEW_4, 6.15, 280);

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
lemlib::ControllerSettings lateral_controller(12, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3.2, // derivative gain (kD)
                                            3, // anti-windup
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
                                             300, // large error range timeout, in milliseconds
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


//move: 200, stop: 0
void intake_move(int speed)
{
    intake.move_velocity(speed);
}

//true: hold, false: release
void mogo_move(bool x)
{
    mogo_pneumatic.set_value(x);
}

void doinker_move(bool x)
{
    doinker_pneumatic.set_value(x);
}

/*
void relative_drive(lemlib::Pose target_pose)
{
    lemlib::Pose current_pose = chassis.getPose();
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint()
}*/


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

            pros::lcd::print(3, "Horizontal encoder: %i", horizontal_encoder.get_position());
            pros::lcd::print(4, "Vertical encoder: %i", right_motor_group.get_position());

            pros::lcd::print(5, "Horizontal distance: %f", horizontal_tracking_wheel.getDistanceTraveled());
            pros::lcd::print(6, "Vertical distance: %f", vertical_tracking_wheel.getDistanceTraveled());

            // log telemetry sink
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());

            // delay to save resources
            pros::delay(delay_time);
        }
    });
}

void disabled() {}

void competition_initialize() {}

//Path
ASSET(draft1_txt);
ASSET(draft2_txt);
ASSET(example_txt);
ASSET(final_path_1_txt);
ASSET(final_path_2_txt);


void autonomous()
{
    //start
    chassis.setPose(-60, 0, 90);
    //score the alliance wall stake
    intake_move(200);
    pros::delay(600);
    //move forward and turn 90
    chassis.moveToPoint(-47, 0, 1000);
    chassis.turnToHeading(0, 1000);
    //move to the mogo and grasp
    chassis.moveToPoint(-46.5, -16.5, 2000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(400);
    //follow path
    chassis.turnToHeading(120, 1000);
    chassis.follow(final_path_1_txt, 18, 5000);
    chassis.waitUntilDone();
    //move backward to score ladybrown
    chassis.moveToPoint(4.25, -45, 1000, {.forwards = false});
    chassis.waitUntilDone();
    //hold ladybrown
    ladybrown_target_set(20);
    chassis.turnToHeading(180, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(1.5, -63.5, 1000);
    chassis.turnToHeading(189, 1000);
    chassis.waitUntilDone();
    pros::delay(100);
    intake_move(0);
    intake.brake();
    ladybrown_target_set(130);
    pros::delay(1000);
    ladybrown_target_set(0);
    ladybrown.brake();
    chassis.setPose(0, -58, 180);
    chassis.moveToPoint(0, -46.5, 1000, {.forwards = false});
    intake_move(200);
    chassis.turnToHeading(-90, 1000, {.minSpeed = 10, .earlyExitRange = 30});
    chassis.follow(final_path_2_txt, 18, 5000);
    chassis.turnToHeading(135, 1000, {.minSpeed = 10, .earlyExitRange = 45});
    chassis.moveToPoint(-44, -64, 1000);
    pros::delay(600);
    chassis.moveToPose(-62, -68, -45, 1000, {.forwards = false});
    intake_move(-100);
    mogo_move(false);

    pros::delay(delay_time);
}

void opcontrol() {
    while (true)
    {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 24, 2000);
            chassis.waitUntilDone();
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -24, 2000, {.forwards = false});
            chassis.waitUntilDone();
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 2000);
            chassis.waitUntilDone();
            master.rumble(".");
        }
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(-90, 2000);
            chassis.waitUntilDone();
            master.rumble(".");
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A))
        {
            ladybrown_target_set(130);
            pros::delay(1000);
            ladybrown_target_set(0);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            ladybrown_target_set(130);
        }

        pros::delay(delay_time);
    }
}