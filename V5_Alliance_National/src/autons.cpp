#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED_1 = 120;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int DRIVE_SPEED_2 = 90;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(20.0, 0.0, 100.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.05, 20.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}


///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED_1);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED_1);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED_1},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED_1},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED_1}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED_1},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED_1},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED_1},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED_1}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED_1},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED_1},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED_1},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED_1},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED_1}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED_1},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

void negative_red() {

  //Step 1
  chassis.pid_drive_set(-10_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, 90);
  Intake_Conveyor(200);
  chassis.pid_wait();
  pros::delay(800);
  Intake_Conveyor(0);

  //Step 2
  chassis.pid_turn_set(-120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 90);
  chassis.pid_wait();
  mogo.set(true);                         
  pros::delay(500);

  //Step 3
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);
  Intake_Conveyor(200);
  chassis.pid_wait();

  //Srep 4
  chassis.pid_turn_set(10_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);          
  chassis.pid_wait();

  //Step 5
  chassis.pid_turn_set(-60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(60_in, 110);          
  chassis.pid_wait();
  pros::delay(1000);
  Intake_Conveyor(0);
}

void positive_red() {
  chassis.pid_drive_set(-30_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-30_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, 110);
  chassis.pid_wait();

  /////mogo 1
  mogo.set(true);                         
  pros::delay(500);
  chassis.pid_turn_set(20_deg, 90);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait();
  Intake_Conveyor(0);
  mogo.set(false);

  chassis.pid_turn_set(-90_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15_in, 110);
  chassis.pid_wait();

  ///// mogo 2
  mogo.set(true);                         
  pros::delay(500);
  chassis.pid_turn_set(60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, 110);
  lift_pneumatic.set(true);
  Intake_Conveyor(200);
  chassis.pid_wait();
  
  ///// touch hang
  Intake_Conveyor(0);
  chassis.pid_turn_set(-160_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 110);
  chassis.pid_wait();
  ladybrown.move_relative(2000, 100);
}

void test_ACE() {
  chassis.pid_drive_set(45_in, 110);
  Intake_Conveyor(200);
  chassis.pid_wait();
  pros::delay(800);
  Intake_Conveyor(0);
  }

/* Version 2
void negative_red(){
  //Step 1
  chassis.pid_drive_set(-10_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, 90);
  Intake_Conveyor(200);
  chassis.pid_wait();
  pros::delay(800);
  Intake_Conveyor(0);

  //Step 2
  chassis.pid_turn_set(-120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 90);
  chassis.pid_wait();
  mogo.set(true);                         
  pros::delay(500);

  //Step 3
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);
  Intake_Conveyor(200);
  chassis.pid_wait();

  //Srep 4
  chassis.pid_turn_set(10_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);          
  chassis.pid_wait();

  //Step 5
  chassis.pid_turn_set(-60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(60_in, 110);          
  chassis.pid_wait();
  pros::delay(1000);
  Intake_Conveyor(0);
}*/


/*
void Auto15s() {
  // move backward to grasp stake
  chassis.pid_drive_set(-24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();

  // grab stake

  // turn 135 degrees clockwise
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();

  // move forward to grab rings
  chassis.pid_drive_set(24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();
  
  //grab rings;
  Intake(200);
  pros::delay(200);
  Intake(0);
}

void Test() {
  chassis.pid_drive_set(-24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();

  mogo.set(false);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(24_in, DRIVE_SPEED_1, true);
  chassis.pid_wait();

  pros::delay(2000);
}
*/
/*
void red_right()
{
  //lift_pneumatic.set(false);
  mogo.set(false); //lift the piston    
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_2);
  mogo.set(true); //piston down
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);
  chassis.pid_wait();
  chassis.pid_turn_set(-65, TURN_SPEED);
  chassis.pid_wait();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(180, TURN_SPEED);

  Intake(0);
  Conveyor(0);
  chassis.pid_wait();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(8_in, DRIVE_SPEED_2);
}

void red_left() {
  //----- Lui -----
  // move forward 48 inches
  mogo.set(false); // lift the piston
  chassis.pid_drive_set(45_in, DRIVE_SPEED_1, true);
  //------ STEP2. lay banh xe ---------- 
  // run intake
  Intake(200); 
  chassis.pid_wait_quick_chain();
  // move backward 12 inches
  chassis.pid_drive_set(-9_in, DRIVE_SPEED_1, true); 
  chassis.pid_wait();
  pros::delay(100);
  
  // stop intake  
  // turn the robot 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED); 
  chassis.pid_wait();
  // move backward 24 inches
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_1, true); 
  chassis.pid_wait_until(-12_in); //old = -10
  //grasp stake
  mogo.set(true); 
  chassis.pid_wait();
  //activate the conveyor motors
  Conveyor(200); 
  pros::delay(1000);
  // move forward 24 inches
  chassis.pid_turn_set(60_deg, TURN_SPEED, true); 
  chassis.pid_wait();
  //stop conveyor
  chassis.pid_drive_set(13_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  //move ladybrown to touch the hang
  ladybrown.move_relative(2000, 150);
  pros::delay(2000);
  Conveyor(0); 
  Intake(0);
  ladybrown.move_relative(-2000, 150);
}

void blue_right() {
  //----- Lui -----
  // move forward 48 inches
  mogo.set(false); // lift the piston
  chassis.pid_drive_set(45_in, DRIVE_SPEED_1, true);
  //------ STEP2. lay banh xe ---------- 
  // run intake
  Intake(200); 
  chassis.pid_wait_quick_chain();
  // move backward 12 inches
  chassis.pid_drive_set(-9_in, DRIVE_SPEED_1, true); 
  chassis.pid_wait();
  pros::delay(100);
  
  // stop intake  
  // turn the robot 90 degrees counter-clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED); 
  chassis.pid_wait();
  // move backward 24 inches
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_1, true); 
  chassis.pid_wait_until(-12_in); // old = -10
  //grasp stake
  mogo.set(true); 
  chassis.pid_wait();
  //activate the conveyor motors
  Conveyor(200); 
  pros::delay(1000);
  // move forward 24 inches
  chassis.pid_turn_set(-60_deg, TURN_SPEED, true); 
  chassis.pid_wait();
  //stop conveyor
  chassis.pid_drive_set(13_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  //move ladybrown to touch the hang
  ladybrown.move_relative(2000, 150);
  pros::delay(2000);
  Conveyor(0); 
  Intake(0);
}

void blue_left()
{
  //lift_pneumatic.set(false);
  mogo.set(false); //lift the piston
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_2);
  mogo.set(true); //piston down
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  Intake(0);
  Conveyor(0);
  chassis.pid_wait();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(8_in, DRIVE_SPEED_2);
}

void red_left_ver2(){
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_1);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_2);
  chassis.pid_wait_until(-10_in);
  mogo.set(true);
  chassis.pid_wait();
  chassis.pid_turn_set(60_deg, TURN_SPEED);
  chassis.pid_wait();
  Intake(200);
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_1);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(150_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(23_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  pros::delay(800);
  chassis.pid_drive_set(-23_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(-150_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(23_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  pros::delay(800);
  chassis.pid_drive_set(-23_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(-130_deg, TURN_SPEED);
  chassis.pid_wait();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(42_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  pros::delay(500);
  ladybrown.move_relative(-2000, 150);
}
*/
void red_right_ver_2(){
  mogo.set(false); //lift the piston
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_2);
  mogo.set(true); //piston down
  pros::delay(300);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);  
  chassis.pid_wait();
  chassis.pid_turn_set(65, TURN_SPEED);
  chassis.pid_wait();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED); // old = 25 deg
  chassis.pid_wait();
  mogo.set(false);
  pros::delay(200);
  chassis.pid_drive_set(5_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(300);
  chassis.pid_drive_set(22_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);  // old = 115 deg
  chassis.pid_wait();
  chassis.pid_drive_set(22_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  Intake_Conveyor(0);
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(6_in, DRIVE_SPEED_2);
}

/*
void red_right_ver3()
{
  mogo.set(false); 
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_2);
  mogo.set(true);
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);  
  chassis.pid_wait();
  chassis.pid_turn_set(-65, TURN_SPEED);
  chassis.pid_wait();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait();
  corner_pneumatic.set(true);
  chassis.pid_drive_set(40_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(15_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED_2);
  corner_pneumatic.set(false);
  chassis.pid_wait();
  chassis.pid_drive_set(8_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(180, TURN_SPEED);
  Intake(0);
  Conveyor(0);
}

void red_right_ver4(){
  mogo.set(false);
  corner_pneumatic.set(false);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(-48_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(10_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(7_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  corner_pneumatic.set(true);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  corner_pneumatic.set(false);
  pros::delay(200);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  Intake(200);
  chassis.pid_drive_set(38_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(12_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-30_in, DRIVE_SPEED_1);
  chassis.pid_wait();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED_2);
  chassis.pid_wait();
  mogo.set(true);
  Conveyor(200);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(6_in, DRIVE_SPEED_2);
  Intake(0);
  Conveyor(0);


}
*/