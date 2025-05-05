#include "lemlib/api.hpp" // IWYU pragma: keep
#include "main.h"

//Motors
#define left_motor_port_1 -9
#define left_motor_port_2 -4
#define left_motor_port_3 19
#define right_motor_port_1 10
#define right_motor_port_2 6
#define right_motor_port_3 -8
#define conveyor_port -2
#define intake_port 11
#define ladybrown_port 20

//IMU
#define IMU_port 18

//Encoder
#define horizontal_encoder_port -3
#define vertical_encoder_port 5
#define ladybrown_encoder_port 7

//Pistons
#define mogo_pneumatic_port 'B'
#define left_doinker_pneumatic_port 'H'
#define right_doinker_pneumatic_port 'A'

//Ladybrown
#define ladybrown_hold_angle 23.5
#define ladybrown_up_angle 150
#define ladybrown_down_angle 1

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
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, 0.9);

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
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

//Two PID controllers: lateral and angular
//20 0 100 is good
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            125, // derivative gain (kD)
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

my_custom_PID ladybrown_PID(4, 0, 4);
double rotation_error;
int rotation_count;
bool move_done = true;
long long failsafe_count = 0;

void ladybrown_move_PID(double target, double deadband, int time_count)
{
    rotation_count = 0;
	ladybrown_PID.reset();
    move_done = false;
    failsafe_count = 0;

	while (!move_done)
	{
		rotation_error = target - ((double)(ladybrown_encoder.get_position())/100);
		ladybrown.move_velocity(ladybrown_PID.update(rotation_error));

		if (std::abs(rotation_error) < deadband) rotation_count++;
        if (rotation_count >= time_count) move_done = true;
        if (target > 140 && ((double)(ladybrown_encoder.get_position())/100) > 80 && ladybrown.get_actual_velocity() < 50) failsafe_count++;
        if (failsafe_count > 60) move_done = true;

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


bool enable_antijam = false;
long long antijam_count = 0;
void conveyor_antijam()
{
    while (true)
    {
        if (enable_antijam)
        {
            if (conveyor.get_target_velocity() - conveyor.get_actual_velocity() > 300) antijam_count++;
            if (conveyor.get_target_velocity() - conveyor.get_actual_velocity() <= 300) antijam_count = 0;
            if (antijam_count > 25)
            {
                antijam_count = 0;
                conveyor_move(-300);
                intake_move(-200);
                pros::delay(500);
                conveyor_move(600);
                intake_move(200);
            }
            pros::delay(delay_time);
        }
        else antijam_count = 0;
        pros::delay(delay_time);
    }
}
pros::Task Conveyor_Antijam(conveyor_antijam);



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
            pros::lcd::print(3, "kP: %f", lateral_controller.kP); // lateral
            pros::lcd::print(4, "kD: %f", lateral_controller.kD);
            
            if (record_speed)
            {
                //for angular
                std::cout <<(imu.get_rotation() - current_heading) <<",\n";
                //for lateral
                //std::cout <<chassis.getPose().y <<",\n";
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


//blue alliance wall
void autonomous()
{    
    //start
    chassis.setPose(-60, 0, 90);

    //score the alliance wall stake
    intake_move(200);
    conveyor_move(600);
    pros::delay(600);

    //move forward and turn 90
    chassis.moveToPoint(-45.4, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();

    //move to the mogo and grasp
    chassis.moveToPoint(-45.7, -18.5, 2000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(300);

    //follow path and score rings
    chassis.turnToHeading(113, 700);
    enable_antijam = true;
    chassis.follow(final_path_1_txt, 30, 5000);
    chassis.waitUntilDone();
    pros::delay(500);
    
    //move to the wall stake
    chassis.moveToPoint(2.8, -47, 2000, {.forwards = false}); //x: 3.85
    chassis.waitUntilDone();
    enable_antijam = false;
    chassis.turnToHeading(180, 600, {.minSpeed = 20});
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.waitUntilDone();
    chassis.moveToPoint(4.5, -66.75, 2000); //x:4.45, y:67
    chassis.waitUntilDone();
    pros::delay(650);

    //score ladybrown
    intake.brake();
    conveyor.brake();
    ladybrown_move_PID(ladybrown_up_angle, 5, 1);

    //take in 3 rings in a row
    chassis.moveToPoint(3, -46, 1000, {.forwards = false});
    ladybrown_move_PID(ladybrown_down_angle, 4, 1);
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    enable_antijam = true;
    chassis.moveToPoint(-60, -53, 10000, {.maxSpeed = 65});
    chassis.waitUntilDone();
    pros::delay(700);

    //take in the final ring
    chassis.moveToPose(-37.5, -74.5, 135, 1100);
    chassis.waitUntilDone();
    pros::delay(300);

    //move to the corner
    chassis.moveToPose(-70, -91, 45, 550, {.forwards = false, .minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(200);
    mogo_move(false);
    enable_antijam = false;
    pros::delay(600);
    conveyor.brake();
    intake.brake();

    //move out of the corner
    chassis.moveToPoint(-30, -40, 2000);
    chassis.waitUntilDone();

    //move to the mobile goal
    chassis.moveToPoint(-43.5, 10, 3000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-43.5, 18, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    enable_antijam = true;
    intake_move(200);
    conveyor_move(600);
    mogo_move(true);
    pros::delay(200);

    //follow path and intake rings
    chassis.turnToHeading(80, 1000);
    chassis.waitUntilDone();
    chassis.follow(final_path_4_txt, 30, 5000);
    chassis.waitUntilDone();

    //move to the ladybrown
    chassis.moveToPoint(5.15, 38, 4000, {.forwards = false}); //x:5.2
    chassis.waitUntilDone();
    enable_antijam = false;
    chassis.turnToHeading(0, 1000, {.minSpeed = 40});
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.waitUntilDone();
    chassis.moveToPoint(3.85, 59.75, 2000); //x:3.9
    chassis.waitUntilDone();
    pros::delay(700);

    //score ladybrown
    intake.brake();
    conveyor.brake();
    ladybrown_move_PID(ladybrown_up_angle, 5, 1);
    chassis.moveToPoint(7.8, 39.5, 2000, {.forwards = false});
    ladybrown_move_PID(ladybrown_down_angle, 4, 1);
    chassis.waitUntilDone();

    //intake 3 rings in a row
    intake_move(200);
    conveyor_move(600);
    enable_antijam = true;
    chassis.moveToPoint(-56, 45.25, 5000, {.maxSpeed = 65}); //x:45
    chassis.waitUntilDone();
    pros::delay(1000);

    //take in the final ring
    conveyor.brake();
    enable_antijam = false;
    chassis.moveToPose(-35, 67.5, 45, 1100);
    chassis.waitUntilDone();

    //move to the corner
    chassis.moveToPose(-67.5, 80, 135, 700, {.forwards = false, .minSpeed = 80}); //y:84
    chassis.waitUntilDone();
    conveyor_move(-300);
    mogo_move(false);
    pros::delay(200);
    conveyor.brake();

    //follow path and move the ring to the ladybrown
    chassis.moveToPoint(4, 44.5, 3000, {.minSpeed = 80, .earlyExitRange = 12});
    ladybrown_move_PID(ladybrown_hold_angle, 0.5, 3);
    chassis.waitUntil(20);
    conveyor_move(600);
    chassis.waitUntilDone();
    chassis.moveToPoint(42, 10, 2000); //(42; 10)
    chassis.waitUntilDone();

    //grab the mobile goal
    chassis.turnToHeading(-40, 1000, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(52, -6, 3000, {.forwards = false, .maxSpeed = 80}); //(52; -6)
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(200);

    //move to the alliance wall stake

    //direct
    // chassis.turnToHeading(90, 1000, {.minSpeed = 40});
    // chassis.waitUntilDone();
    // chassis.moveToPose(60.3, -4.5, 90, 1500); //(60.3, -4.5)
    // conveyor.brake();

    //passive
    chassis.turnToHeading(90, 1000, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(72, -4.5, 2000);
    chassis.waitUntilDone();
    conveyor.brake();
    lemlib::Pose current = chassis.getPose();
    double distance = 9;
    ladybrown_move_PID(185, 4, 1);
    chassis.moveToPose(current.x - sin(distance), current.y - abs(cos(distance)), current.theta, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    
    //score on the alliance wall stake
    
    chassis.waitUntilDone();
    chassis.moveToPoint(52, -6, 2000, {.forwards = false});
    intake_move(200);
    conveyor_move(600);
    chassis.waitUntilDone();

    //take in the first ring
    chassis.moveToPoint(30, -28, 3000);
    ladybrown_move_PID(130, 5, 1);
    chassis.waitUntilDone();
    pros::delay(200);

    //take in the second ring
    enable_antijam = true;
    chassis.turnToHeading(135, 800, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(52, -52, 3500); //52, -47
    chassis.waitUntilDone();
    pros::delay(200);

    //take in the third ring
    chassis.moveToPoint(63, -52, 3500); //59, -47
    chassis.waitUntilDone();

    //sweep the rings out
    left_doinker_move(true);
    pros::delay(500);
    enable_antijam = false;
    intake.brake();
    conveyor.brake();
    chassis.moveToPoint(47, -60, 2000);//x:-60
    chassis.waitUntilDone();
    left_doinker_move(false);

    //move to the corner 1
    chassis.moveToPose(75, -80, -45, 1500, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(200);
    mogo_move(false);
    pros::delay(200);

    //move out of the corner 1
    chassis.moveToPoint(40, -40, 5000);
    chassis.waitUntilDone();
    mogo_move(true);

    //move to the corner 2
    chassis.moveToPoint(74, 54, 3000, {.forwards = false});
    ladybrown_move_PID(240, 5, 1);
    chassis.waitUntilDone();


    //move out of the corner 2
    chassis.moveToPoint(40, 33, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 800, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPose(0, -7, 45, 5000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
}



/*
void autonomous()
{    
    //start
    chassis.setPose(-60, 0, 90);

    //score the alliance wall stake
    intake_move(200);
    conveyor_move(600);
    pros::delay(600);

    //move forward and turn 90
    chassis.moveToPoint(-45.4, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 700);
    chassis.waitUntilDone();

    //move to the mogo and grasp
    chassis.moveToPoint(-45.7, -18.5, 2000, {.forwards = false});
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(300);

    //follow path and score rings
    chassis.turnToHeading(113, 700);
    enable_antijam = true;
    chassis.follow(final_path_1_txt, 30, 5000);
    chassis.waitUntilDone();
    pros::delay(500);
    
    //move to the wall stake
    chassis.moveToPoint(2.8, -47, 2000, {.forwards = false}); //x: 3.85
    chassis.waitUntilDone();
    enable_antijam = false;
    chassis.turnToHeading(180, 600, {.minSpeed = 20});
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.waitUntilDone();
    chassis.moveToPoint(4.5, -66.75, 2000); //x:4.45, y:67
    chassis.waitUntilDone();
    pros::delay(650);

    //score ladybrown
    intake.brake();
    conveyor.brake();
    ladybrown_move_PID(ladybrown_up_angle, 5, 1);

    //take in 3 rings in a row
    chassis.moveToPoint(3, -46, 1000, {.forwards = false});
    ladybrown_move_PID(ladybrown_down_angle, 4, 1);
    chassis.waitUntilDone();
    intake_move(200);
    conveyor_move(600);
    enable_antijam = true;
    chassis.moveToPoint(-60, -53, 10000, {.maxSpeed = 65});
    chassis.waitUntilDone();
    pros::delay(700);

    //take in the final ring
    chassis.moveToPose(-37.5, -74.5, 135, 1100);
    chassis.waitUntilDone();
    pros::delay(300);

    //move to the corner
    chassis.moveToPose(-70, -91, 45, 550, {.forwards = false, .minSpeed = 80});
    chassis.waitUntilDone();
    pros::delay(200);
    mogo_move(false);
    enable_antijam = false;
    pros::delay(600);
    conveyor.brake();
    intake.brake();

    //move out of the corner
    chassis.moveToPoint(-30, -40, 2000);
    chassis.waitUntilDone();

    //move to the mobile goal
    chassis.moveToPoint(-43.5, 10, 3000, {.forwards = false, .minSpeed = 50, .earlyExitRange = 10});
    chassis.waitUntilDone();
    chassis.moveToPoint(-43.5, 18, 1000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    enable_antijam = true;
    intake_move(200);
    conveyor_move(600);
    mogo_move(true);
    pros::delay(200);

    //follow path and intake rings
    chassis.turnToHeading(80, 1000);
    chassis.waitUntilDone();
    chassis.follow(final_path_4_txt, 30, 5000);
    chassis.waitUntilDone();

    //move to the ladybrown
    chassis.moveToPoint(5.15, 38, 4000, {.forwards = false}); //x:5.2
    chassis.waitUntilDone();
    enable_antijam = false;
    chassis.turnToHeading(0, 1000, {.minSpeed = 40});
    ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
    chassis.waitUntilDone();
    chassis.moveToPoint(3.85, 59.75, 2000); //x:3.9
    chassis.waitUntilDone();
    pros::delay(700);

    //score ladybrown
    intake.brake();
    conveyor.brake();
    ladybrown_move_PID(ladybrown_up_angle, 5, 1);
    chassis.moveToPoint(7.8, 39.5, 2000, {.forwards = false});
    ladybrown_move_PID(ladybrown_down_angle, 4, 1);
    chassis.waitUntilDone();

    //intake 3 rings in a row
    intake_move(200);
    conveyor_move(600);
    enable_antijam = true;
    chassis.moveToPoint(-56, 45.25, 5000, {.maxSpeed = 65}); //x:45
    chassis.waitUntilDone();
    pros::delay(1000);

    //take in the final ring
    conveyor.brake();
    enable_antijam = false;
    chassis.moveToPose(-35, 67.5, 45, 1100);
    chassis.waitUntilDone();

    //move to the corner
    chassis.moveToPose(-67.5, 80, 135, 700, {.forwards = false, .minSpeed = 80}); //y:84
    chassis.waitUntilDone();
    conveyor_move(-300);
    mogo_move(false);
    pros::delay(200);
    conveyor.brake();

    //follow path and move the ring to the ladybrown
    chassis.moveToPoint(4, 44.5, 3000, {.minSpeed = 80, .earlyExitRange = 12});
    conveyor_move(50);
    chassis.waitUntil(15);
    conveyor.brake();
    chassis.waitUntilDone();
    chassis.moveToPoint(42, 10, 2000); //(42; 10)
    chassis.waitUntilDone();

    //grab the mobile goal
    chassis.turnToHeading(-40, 1000, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(52, -6, 3000, {.forwards = false, .maxSpeed = 80}); //(52; -6)
    chassis.waitUntilDone();
    mogo_move(true);
    pros::delay(200);
    conveyor_move(600);
    pros::delay(800);

    //take in the first ring
    chassis.moveToPoint(30, -28, 3000);
    ladybrown_move_PID(130, 5, 1);
    chassis.waitUntilDone();
    pros::delay(200);

    //take in the second ring
    enable_antijam = true;
    chassis.turnToHeading(135, 800, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPoint(52, -52, 3500); //52, -47
    chassis.waitUntilDone();
    pros::delay(200);

    //take in the third ring
    chassis.moveToPoint(63, -52, 3500); //59, -47
    chassis.waitUntilDone();

    //sweep the rings out
    left_doinker_move(true);
    pros::delay(500);
    enable_antijam = false;
    intake.brake();
    conveyor.brake();
    chassis.moveToPoint(47, -60, 2000);//x:-60
    chassis.waitUntilDone();
    left_doinker_move(false);

    //move to the corner 1
    chassis.moveToPose(75, -80, -45, 1000, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();
    mogo_move(false);
    pros::delay(200);
    chassis.moveToPoint(47, -60, 2000);
    mogo_move(true);
    chassis.waitUntilDone();
    chassis.moveToPose(75, -80, -45, 1000, {.forwards = false, .minSpeed = 90});
    chassis.waitUntilDone();

    //move out of the corner 1
    chassis.moveToPoint(45, -45, 5000);
    chassis.waitUntilDone();
    mogo_move(true);

    //move to the corner 2
    chassis.moveToPoint(74, 54, 3000, {.forwards = false});
    ladybrown_move_PID(240, 5, 1);
    chassis.waitUntilDone();

    //move out of the corner 2
    chassis.moveToPoint(40, 33, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(45, 800, {.minSpeed = 40});
    chassis.waitUntilDone();
    chassis.moveToPose(0, -7, 45, 5000, {.forwards = false, .maxSpeed = 90});
    chassis.waitUntilDone();
}
*/

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