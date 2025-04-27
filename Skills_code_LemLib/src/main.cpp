//TUNING PID: TUNING P GAIN AND D GAIN THROUGH ANGULAR MOVEMENT

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"

//Motors
#define left_motor_port_1 -9
#define left_motor_port_2 -5
#define left_motor_port_3 19
#define right_motor_port_1 10
#define right_motor_port_2 7
#define right_motor_port_3 -8
#define conveyor_port -2
#define intake_port 11
#define ladybrown_port 20

//IMU
#define IMU_port 18

//Encoder
#define horizontal_encoder_port -4
#define vertical_encoder_port 6
#define ladybrown_encoder_port 1

//Pistons
#define mogo_pneumatic_port 'B'
#define left_doinker_pneumatic_port 'H'
#define right_doinker_pneumatic_port 'A'

//Ladybrown
#define ladybrown_deadband 2
#define ladybrown_hold_angle 24.5
#define ladybrown_up_angle 150
#define ladybrown_down_angle 2

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
pros::MotorGroup left_motor_group({left_motor_port_1, left_motor_port_2, left_motor_port_3}, pros::MotorGearset::blue);
pros::MotorGroup right_motor_group({right_motor_port_1, right_motor_port_2, right_motor_port_3}, pros::MotorGearset::blue);
pros::Motor intake(intake_port, pros::MotorGearset::green);
pros::Motor conveyor(conveyor_port, pros::MotorGearset::blue);
pros::Motor ladybrown(ladybrown_port, pros::MotorGearset::green);

//IMU AND ROTATION SENSORS
pros::IMU imu(IMU_port);
pros::Rotation ladybrown_encoder(ladybrown_encoder_port);
pros::Rotation horizontal_encoder(horizontal_encoder_port);
pros::Rotation vertical_encoder(vertical_encoder_port);

//PNEUMATICS
pros::ADIDigitalOut mogo_pneumatic(mogo_pneumatic_port);
pros::ADIDigitalOut left_doinker_pneumatic(left_doinker_pneumatic_port);
pros::ADIDigitalOut right_doinker_pneumatic(right_doinker_pneumatic_port);





//Tracking wheels, wheel type and offset to be determined
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, 1.4);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.9); //phai tune lai

//Drivetrain settings
lemlib::Drivetrain drivetrain(&left_motor_group, 
							  &right_motor_group,
							  11.02, //track width
							  lemlib::Omniwheel::NEW_325, //wheel type
							  400, //rpm
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
//20 0 100 is good
lemlib::ControllerSettings lateral_controller(25, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            127, // derivative gain (kD)
                                            3, // anti-windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

//7 0 57 is good
lemlib::ControllerSettings angular_controller(1.5, //. proportional gain (kP)
                                             0, // integral gain (kI)
                                             9, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             200, // large error range timeout, in milliseconds
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

my_custom_PID ladybrown_PID(4, 0, 12);
double rotation_error;
int rotation_count;
bool move_done = true;

void ladybrown_move_PID(double target, double deadband, int time_count)
{
    rotation_count = 0;
	ladybrown_PID.reset();
    move_done = false;

	while (!move_done)
	{
		rotation_error = target - ((double)(ladybrown_encoder.get_position())/100);
		ladybrown.move_velocity(ladybrown_PID.update(rotation_error));

		if (std::abs(rotation_error) < deadband) rotation_count++;
        if (rotation_count >= time_count) move_done = true;

		pros::delay(delay_time);
    }
    ladybrown.brake();
    ladybrown_PID.reset();
}

void intake_move(int speed)
{
    intake.move_velocity(speed);
}

void conveyor_move(int speed)
{
    conveyor.move_velocity(speed);
}

void mogo_move(bool x)
{
    mogo_pneumatic.set_value(x);
}

void left_doinker_move(bool x)
{
    left_doinker_pneumatic.set_value(x);
}

void right_doinker_move(bool x)
{
    right_doinker_pneumatic.set_value(x);
}



bool record_speed = false;
double current_heading;

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
    conveyor.tare_position();
    ladybrown.tare_position();
    vertical_encoder.reset_position();
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
            pros::lcd::print(3, "kP: %f", lateral_controller.kP); // heading
            pros::lcd::print(4, "kD: %f", lateral_controller.kD);
            
            if (record_speed)
            {
                //for angular
                //std::cout <<(imu.get_rotation() - current_heading) <<",\n";
                //for lateral
                std::cout <<chassis.getPose().y <<",\n";
                //std::cout <<left_motor_group.get_actual_velocity() <<",\n";
                pros::delay(delay_time);
            }

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
ASSET(final_path_4_txt);
ASSET(final_path_5_txt);

void autonomous()
{    
    //start
    chassis.setPose(-60, 0, 90);

    //score the alliance wall stake
    intake_move(200);
    conveyor_move(600);
    pros::delay(600);

    //move forward and turn 90
    chassis.moveToPoint(-45.75, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 750);
    chassis.waitUntilDone();

    //move to the mogo and grasp
    chassis.moveToPoint(-47.25, -17, 2000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(300);

    //follow path and score rings
    chassis.turnToHeading(113, 800);
    chassis.follow(final_path_1_txt, 25, 5000);
    chassis.waitUntilDone();
    pros::delay(200);
    
    //move to the alliance stake
    chassis.moveToPoint(4.15, -45, 800, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.waitUntilDone();
    chassis.moveToPoint(4.05, -68, 2000);
    chassis.waitUntilDone();
    pros::delay(800);

    //score ladybrown
    intake.brake();
    conveyor.brake();

    ladybrown_move_PID(ladybrown_up_angle, 5, 1);
    ladybrown_move_PID(ladybrown_down_angle, 5, 1);
    ladybrown.brake();

    //take in 3 rings in a row
    chassis.moveToPoint(3, -46, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    chassis.moveToPoint(-60, -55, 10000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    // chassis.moveToPoint(-55, -55, 1000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(-45, -66, 2000);
    // chassis.waitUntilDone();
    // chassis.moveToPose(-61, -63, -45, 5000, {.forwards = false});
    // chassis.waitUntilDone();
    // mogo_move(false);
    // conveyor.brake();

    //take in the final ring
    chassis.moveToPose(-37.5, -74.5, 135, 1200);
    chassis.waitUntilDone();
    pros::delay(500);

    //move to the corner
    chassis.moveToPose(-66, -76, 45, 900, {.forwards = false});
    chassis.waitUntilDone();
    pros::delay(200);
    mogo_move(false);
    conveyor.brake();
    intake.brake();

    //move our of the corner
    chassis.moveToPoint(-30, -40, 2000);
    chassis.waitUntilDone();

    //move to the mobile goal
    chassis.moveToPoint(-41, 18, 10000, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    mogo_move(true);
    pros::delay(200);
    chassis.turnToHeading(80, 1000);
    chassis.waitUntilDone();
    chassis.follow(final_path_4_txt, 20, 20000);
    chassis.waitUntilDone();

    /*
    intake_move(-200);
    conveyor_move(-200);
    chassis.waitUntil(5);
    intake_move(200);
    conveyor_move(600);
    chassis.waitUntilDone();
    pros::delay(600);

    //take in the final ring
    chassis.moveToPose(-40, -72, 135, 1200);
    chassis.waitUntilDone();
    pros::delay(200);

    //move to the corner
    chassis.moveToPose(-62.5, -71.5, -45, 1500, {.forwards = false});
    chassis.waitUntilDone();
    conveyor_move(-200);
    mogo_move(false);
    pros::delay(300);
    intake.brake();
    conveyor.brake();

    //move to the center
    chassis.follow(final_path_3_txt, 20, 20000);
    chassis.waitUntil(70);
    intake_move(200);
    conveyor_move(600);
    chassis.waitUntilDone();

    //push the ring
    chassis.turnToHeading(135, 700);
    chassis.waitUntil(3);
    intake.brake();
    conveyor.brake();
    chassis.waitUntilDone();
    chassis.moveToPoint(-24, 24, 2000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, 20, 1000);
    chassis.waitUntilDone();

    //go grab the mogo
    chassis.turnToHeading(97, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-52, 27.5, 3000, {.forwards = false, .maxSpeed = 75});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(100);

    //go pure pursuit and intake rings
    chassis.turnToHeading(55, 700);
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    chassis.follow(final_path_4_txt, 20, 20000);
    chassis.waitUntilDone();

    //move back to score the wall stake
    chassis.moveToPoint(4.5, 48.2, 4000, {.forwards = false});
    chassis.waitUntilDone();
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.turnToHeading(0, 800);
    chassis.waitUntilDone();
    chassis.moveToPoint(5.5, 61, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 500);
    pros::delay(700);


    //score ladybrown
    intake.brake();
    conveyor.brake();
    intake.brake();
    ladybrown_move_PID(ladybrown_up_angle, 3, 1);
    pros::delay(1200);
    ladybrown_move_PID(ladybrown_down_angle, 3, 1);
    ladybrown.brake();
    pros::delay(400);

    //score 3 rings in a row
    chassis.moveToPoint(3.4, 43, 4000, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    chassis.moveToPoint(-58, 52, 10000, {.maxSpeed = 70});
    intake_move(-200);
    conveyor_move(-200);
    chassis.waitUntil(5);
    intake_move(200);
    conveyor_move(600);
    chassis.waitUntilDone();
    pros::delay(500);

    //move to the corner
    chassis.moveToPose(-60, 72, -45, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(-200);
    conveyor_move(-200);
    mogo_move(false);
    pros::delay(800);
    intake.brake();
    conveyor.brake();

    //follow path and store 1 ring
    intake_move(80);
    conveyor_move(80);
    chassis.follow(final_path_5_txt, 20, 10000);
    chassis.waitUntilDone();
    intake.brake();
    conveyor.brake();

    //move backwards to and grab the mobile goal
    //chassis.setPose(24.7, 10.4, -205.2);
    chassis.turnToHeading(-43, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(43, -10, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    mogo_move(true); 
    pros::delay(200);

    //put the ring in and swing the mobile goal
    intake_move(200);
    conveyor_move(600);
    pros::delay(600);
    chassis.turnToHeading(75, 1000);
    chassis.waitUntilDone();
    mogo_move(false);

    //turn and move to the corner
    chassis.turnToHeading(-30, 1000);
    chassis.waitUntilDone();
    mogo_move(true);
    intake.brake();
    conveyor.brake();
    chassis.moveToPoint(75, -90, 2500, {.forwards = false});
    chassis.waitUntilDone();

    //move to the holding position
    chassis.moveToPoint(42, -24, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-150, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(75, 90, 2500, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(45, 45, 2000);
    chassis.waitUntilDone();
    */
}


void opcontrol() 
{
    while (true)
    {
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.arcade(leftY, rightX);

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            current_heading = imu.get_rotation();
            record_speed = true;
            pros::delay(200);
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(358, 3000, {.direction = AngularDirection::CW_CLOCKWISE});
            chassis.waitUntilDone();
            master.rumble(".");
            pros::delay(500);
            record_speed = false;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
        {
            record_speed = true;
            pros::delay(200);
            chassis.turnToHeading(2, 3000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE});
            chassis.waitUntilDone();
            master.rumble(".");
            pros::delay(500);
            record_speed = false;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1))
        {
            // left_motor_group.move_velocity(600);
            // right_motor_group.move_velocity(600);
            // pros::delay(1000);
            // left_motor_group.move_velocity(300);
            // right_motor_group.move_velocity(300);
            record_speed = true;
            pros::delay(200);
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 48, 5000);
            chassis.waitUntilDone();
            master.rumble(".");
            pros::delay(500);
            record_speed = false;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
        {
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -24, 5000, {.forwards = false});
            chassis.waitUntilDone();
            master.rumble(".");
            pros::delay(500);
            record_speed = false;
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
        {
            lateral_controller.kP += 0.5;
            pros::delay(200);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))
        {
            lateral_controller.kP -= 0.5;
            pros::delay(200);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X))
        {
            lateral_controller.kD += 0.5;
            pros::delay(200);
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B))
        {
            lateral_controller.kD += 0.5;
            pros::delay(200);
        }

        pros::delay(delay_time);
    }
}