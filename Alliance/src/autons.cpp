#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int SLOW_DRIVE_SPEED = 75;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

#define DRIVE_SPEED_1 120
#define DRIVE_SPEED_2 90

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


void negative_red() {
  chassis.drive_angle_set(180);

  Intake_Conveyor(50);
  pros::delay(100);
  Intake_Conveyor(0);

  chassis.pid_drive_set(-15_in, 110);
  mogo.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in, 90);
  mogo.set(false);
  Intake_Conveyor(200);
  chassis.pid_wait();
  pros::delay(200);

  chassis.pid_turn_set(-50_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(11_in, 110);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(ez::LEFT_SWING, -10_deg, 95, 40);
  chassis.pid_wait_quick_chain();

  chassis.pid_swing_set(ez::LEFT_SWING, -55_deg, 95, 0);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, 90);
  chassis.pid_wait();
  pros::delay(300);

  chassis.pid_drive_set(-21_in, 110);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, 90);
  pros::delay(100);
  chassis.pid_drive_set(20_in, 110);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, 110);
  chassis.pid_wait_quick_chain();
}

void huyanh_red_pos_1()
{
  Intake_Conveyor(50);
  pros::delay(100);
  Intake_Conveyor(0);

  //move backward and turn to score alliance stake
  chassis.drive_angle_set(90_deg);
  mogo.set(false);
  chassis.pid_drive_set(-8.8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-0.1_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);

  //move to the mobile goal
  chassis.pid_swing_set(ez::LEFT_SWING, 42_deg, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-28_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-4.5_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();

  //grab the mobile goal and take in the ring
  mogo.set(true);
  pros::delay(200);
  chassis.pid_turn_set(75_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(29_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);

  //touch the hang
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(35_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void huyanh_blue_pos_1()
{
  //run intake for a short time
  Intake_Conveyor(50);
  pros::delay(100);
  Intake_Conveyor(0);

  //move backward and turn to score alliance stake
  chassis.drive_angle_set(-90_deg);
  mogo.set(false);
  chassis.pid_drive_set(-8.4_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-0.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);

  //move to the mobile goal
  chassis.pid_drive_set(2_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_turn_set(140_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-27.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-4_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();

  //grab the mobile goal and take in the ring
  mogo.set(true);
  pros::delay(200);
  chassis.pid_turn_set(-85_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);

  //touch the hang
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  chassis.pid_wait();
  Intake_Conveyor(0);
  chassis.pid_turn_set(95_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void huyanh_blue_pos_2()
{
  //rush to the mobile goal and take in the ring
  chassis.drive_angle_set(-30_deg);
  Intake_Conveyor(200);
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait_until(35);

  //grab the mobile goal and move back slowly
  doinker_pneumatic.set(true);
  Intake_Conveyor(0);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_drive_set(-10_in, 35);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);

  //turn and score into the mobile goal and move forward
  chassis.pid_turn_set(170_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
  Intake_Conveyor(200);
  pros::delay(100);
  chassis.pid_drive_set(6_in, DRIVE_SPEED);
  chassis.pid_wait();

  //release the mobile goal and grab the other one
  mogo.set(false);
  chassis.pid_turn_set(-97_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_drive_set(-13_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(100);

  //turn and take in the preload ring
  chassis.pid_turn_set(145, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();

  //go touch the hang
  chassis.pid_turn_set(45, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
}

void huyanh_red_pos_1_test()
{
  //run intake for a short time
  Intake_Conveyor(50);
  pros::delay(100);
  Intake_Conveyor(0);

  //move backward and turn to score alliance stake
  chassis.drive_angle_set(90_deg);
  mogo.set(false);
  chassis.pid_drive_set(-8.8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-0.1_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);

  //move to the mobile goal
  chassis.pid_swing_set(ez::LEFT_SWING, 42_deg, SWING_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_turn_set(-145_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-28_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-4.5_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();

  //grab the mobile goal and take in the ring
  mogo.set(true);
  pros::delay(200);
  chassis.pid_turn_set(75_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(29_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(300);

  //go to the corner
  chassis.pid_drive_set(-25_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(35_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(120_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  //sweep the corner, take in a ring and touch the hang
  doinker_pneumatic.set(true);
  chassis.pid_turn_set(-70_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  doinker_pneumatic.set(false);
  chassis.pid_turn_set(-20_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
}


void huyanh_blue_pos_1_test()
{
  //run intake for a short time
  Intake_Conveyor(50);
  pros::delay(100);
  Intake_Conveyor(0);

  //move backward and turn to score alliance stake
  chassis.drive_angle_set(-90_deg);
  mogo.set(false);
  chassis.pid_drive_set(-8.4_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-0.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  pros::delay(500);

  //move to the mobile goal
  chassis.pid_drive_set(2_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_turn_set(140_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-27.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-4_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();

  //grab the mobile goal and take in the ring
  mogo.set(true);
  pros::delay(200);
  chassis.pid_turn_set(-85_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-165_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(27_in, DRIVE_SPEED);
  chassis.pid_wait_until(15_in);
  doinker_pneumatic.set(true);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);

  //go and sweep the corner
  chassis.pid_turn_set(30_deg, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);

  //go touch the hang
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
}

void huyanh_blue_pos_2_test()
{
  //rush to the mobile goal and take in the ring
  chassis.drive_angle_set(-30_deg);
  Intake_Conveyor(200);
  chassis.pid_drive_set(36_in, DRIVE_SPEED);
  chassis.pid_wait_until(35);

  //grab the mobile goal and move back slowly
  doinker_pneumatic.set(true);
  Intake_Conveyor(0);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_drive_set(-10_in, 35);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);

  //turn and score into the mobile goal and move forward
  chassis.pid_turn_set(170_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
  Intake_Conveyor(200);
  pros::delay(100);
  chassis.pid_drive_set(6_in, DRIVE_SPEED);
  chassis.pid_wait();

  //release the mobile goal and grab the other one
  mogo.set(false);
  chassis.pid_turn_set(-97_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_drive_set(-13_in, SLOW_DRIVE_SPEED);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(100);

  //turn and take in the preload ring
  chassis.pid_turn_set(145, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(20_in, DRIVE_SPEED);
  chassis.pid_wait();

  //go touch the hang
  chassis.pid_turn_set(45, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);

  //move backward and go the corner
  chassis.pid_drive_set(-10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(28_in, DRIVE_SPEED);
}