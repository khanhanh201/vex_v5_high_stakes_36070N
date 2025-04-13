#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED_HIGH = 120;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int DRIVE_SPEED_LOW = 90;

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

/*
void negative_red() {

  //Step 1
  chassis.pid_drive_set(-10_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  Intake_Conveyor(0);

  //Step 2
  chassis.pid_turn_set(-120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 90);
  chassis.pid_wait_quick_chain();
  mogo.set(true);                         
  pros::delay(500);

  //Step 3
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();

  //Srep 4
  chassis.pid_turn_set(10_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);          
  chassis.pid_wait_quick_chain();

  //Step 5
  chassis.pid_turn_set(-60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(60_in, 110);          
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  Intake_Conveyor(0);
}

void positive_red() {
  chassis.pid_drive_set(-30_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-30_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-12_in, 110);
  chassis.pid_wait_quick_chain();

  /////mogo 1
  mogo.set(true);                         
  pros::delay(500);
  chassis.pid_turn_set(20_deg, 90);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(12_in, 110);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  mogo.set(true);

  chassis.pid_turn_set(-90_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15_in, 110);
  chassis.pid_wait_quick_chain();

  ///// mogo 2
  mogo.set(true);                         
  pros::delay(500);
  chassis.pid_turn_set(60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, 110);
  lift_pneumatic.set(true);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  
  ///// touch hang
  Intake_Conveyor(0);
  chassis.pid_turn_set(-160_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(20_in, 110);
  chassis.pid_wait_quick_chain();
  ladybrown.move_relative(2000, 100);
}

void test_ACE() {
  chassis.pid_drive_set(45_in, 110);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  Intake_Conveyor(0);
  }

void my_test()
{
  chassis.pid_turn_set(90_deg, 110);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_turn_set(180_deg, 110);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_turn_set(270_deg, 110);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_turn_set(0_deg, 110);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_drive_set(60_in, 110);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  chassis.pid_drive_set(-60_in, 110);
  chassis.pid_wait_quick_chain();
}


/* Version 2
void negative_red(){
  //Step 1
  chassis.pid_drive_set(-10_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  Intake_Conveyor(0);

  //Step 2
  chassis.pid_turn_set(-120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 90);
  chassis.pid_wait_quick_chain();
  mogo.set(true);                         
  pros::delay(500);

  //Step 3
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();

  //Srep 4
  chassis.pid_turn_set(10_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);          
  chassis.pid_wait_quick_chain();

  //Step 5
  chassis.pid_turn_set(-60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(60_in, 110);          
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  Intake_Conveyor(0);
}*/

/* Version 2
void negative_red(){
  //Step 1
  chassis.pid_drive_set(-10_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  Intake_Conveyor(0);

  //Step 2
  chassis.pid_turn_set(-120_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, 90);
  chassis.pid_wait_quick_chain();
  mogo.set(true);                         
  pros::delay(500);

  //Step 3
  chassis.pid_turn_set(45_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(25_in, 110);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();

  //Srep 4
  chassis.pid_turn_set(10_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5_in, 90);          
  chassis.pid_wait_quick_chain();

  //Step 5
  chassis.pid_turn_set(-60_deg, 90);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(60_in, 110);          
  chassis.pid_wait_quick_chain();
  pros::delay(1000);
  Intake_Conveyor(0);
}*/


/*
void Auto15s() {
  // move backward to grasp stake
  chassis.pid_drive_set(-24_in, DRIVE_SPEED_HIGH, true);
  chassis.pid_wait_quick_chain();

  // grab stake

  // turn 135 degrees clockwise
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // move forward to grab rings
  chassis.pid_drive_set(24_in, DRIVE_SPEED_HIGH, true);
  chassis.pid_wait_quick_chain();
  
  //grab rings;
  Intake(200);
  pros::delay(200);
  Intake(0);
}

void Test() {
  chassis.pid_drive_set(-24_in, DRIVE_SPEED_HIGH, true);
  chassis.pid_wait_quick_chain();

  mogo.set(true);

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(24_in, DRIVE_SPEED_HIGH, true);
  chassis.pid_wait_quick_chain();

  pros::delay(2000);
}
*/
/*
void red_right()
{
  //lift_pneumatic.set(false);
  mogo.set(true); //lift the piston    
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  mogo.set(true); //piston down
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, TURN_SPEED);

  Intake(0);
  Conveyor(0);
  chassis.pid_wait_quick_chain();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(8_in, DRIVE_SPEED_LOW);
}

void red_left() {
  //----- Lui -----
  // move forward 48 inches
  mogo.set(true); // lift the piston
  chassis.pid_drive_set(45_in, DRIVE_SPEED_HIGH, true);
  //------ STEP2. lay banh xe ---------- 
  // run intake
  Intake(200); 
  chassis.pid_wait_quick_chain();
  // move backward 12 inches
  chassis.pid_drive_set(-9_in, DRIVE_SPEED_HIGH, true); 
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  
  // stop intake  
  // turn the robot 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 24 inches
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_HIGH, true); 
  chassis.pid_wait_until(-12_in); //old = -10
  //grasp stake
  mogo.set(true); 
  chassis.pid_wait_quick_chain();
  //activate the conveyor motors
  Conveyor(200); 
  pros::delay(1000);
  // move forward 24 inches
  chassis.pid_turn_set(60_deg, TURN_SPEED, true); 
  chassis.pid_wait_quick_chain();
  //stop conveyor
  chassis.pid_drive_set(13_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
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
  mogo.set(true); // lift the piston
  chassis.pid_drive_set(45_in, DRIVE_SPEED_HIGH, true);
  //------ STEP2. lay banh xe ---------- 
  // run intake
  Intake(200); 
  chassis.pid_wait_quick_chain();
  // move backward 12 inches
  chassis.pid_drive_set(-9_in, DRIVE_SPEED_HIGH, true); 
  chassis.pid_wait_quick_chain();
  pros::delay(100);
  
  // stop intake  
  // turn the robot 90 degrees counter-clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 24 inches
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_HIGH, true); 
  chassis.pid_wait_until(-12_in); // old = -10
  //grasp stake
  mogo.set(true); 
  chassis.pid_wait_quick_chain();
  //activate the conveyor motors
  Conveyor(200); 
  pros::delay(1000);
  // move forward 24 inches
  chassis.pid_turn_set(-60_deg, TURN_SPEED, true); 
  chassis.pid_wait_quick_chain();
  //stop conveyor
  chassis.pid_drive_set(13_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  //move ladybrown to touch the hang
  ladybrown.move_relative(2000, 150);
  pros::delay(2000);
  Conveyor(0); 
  Intake(0);
}

void blue_left()
{
  //lift_pneumatic.set(false);
  mogo.set(true); //lift the piston
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  mogo.set(true); //piston down
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  Intake(0);
  Conveyor(0);
  chassis.pid_wait_quick_chain();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(8_in, DRIVE_SPEED_LOW);
}

void red_left_ver2(){
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_until(-10_in);
  mogo.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(60_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake(200);
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(150_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(23_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  chassis.pid_drive_set(-23_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-150_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(23_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  pros::delay(800);
  chassis.pid_drive_set(-23_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-130_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(42_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  ladybrown.move_relative(-2000, 150);
}
*/



/*
void red_right_ver3()
{
  mogo.set(true); 
  chassis.pid_drive_set(-32_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  mogo.set(true);
  pros::delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);  
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Conveyor(200);
  chassis.pid_drive_set(35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-18_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  corner_pneumatic.set(true);
  chassis.pid_drive_set(40_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(15_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-8_in, DRIVE_SPEED_LOW);
  corner_pneumatic.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180, TURN_SPEED);
  Intake(0);
  Conveyor(0);
}
*/
/*
void red_right_ver_4(){
  mogo.set(true);
  doinker_pneumatic.set(false);
  chassis.pid_drive_set(-36_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-40_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic
  chassis.pid_drive_set(-6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  chassis.pid_drive_set(7_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);  
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(25_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  ladybrown.move_relative(2000, 150);
  Intake_Conveyor(0);
 
}
*/

/*
void red_right_ver_5(){
  chassis.pid_drive_set(35_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  pros::delay(400);
  Intake_Conveyor(0);
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(18_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(true);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);
  pros::delay(200);
  chassis.pid_turn_set(-165_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-7_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  Intake_Conveyor(200);
}
*/


  /*
  chassis.pid_drive_set(-15_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);
  chassis.pid_drive_set(15_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  pros::delay(250);
  Intake_Conveyor(0);
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(155_deg, TURN_SPEED);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);
}
  */






//------------------ MAIN SECTION ------------------//
/*
void red_right_goal_rush(){
  mogo.set(false);
  Intake_Conveyor(80);
  chassis.pid_drive_set(40_in, DRIVE_SPEED_HIGH-20);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-48_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED_LOW-10);
  chassis.pid_wait();
  doinker_pneumatic.set(true);
  pros::delay(500); 
  chassis.pid_drive_set(-10.7_in, 38);
  chassis.pid_wait();
  doinker_pneumatic.set(false);
  pros::delay(150);
  chassis.pid_turn_set(153_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14.6_in, 60);
  chassis.pid_wait();
  mogo.set(true);
  Intake_Conveyor(200);
  pros::delay(200);
  chassis.pid_drive_set(13.8_in, DRIVE_SPEED_LOW-20);
  chassis.pid_wait();
  pros::delay(200);
  mogo.set(false);
  pros::delay(120);
  Intake_Conveyor(0);
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-19.7_in, 70);
  chassis.pid_wait();
  
  mogo.set(true);
  pros::delay(500);
  // gap xong stake 2
  Intake_Conveyor(200);
  chassis.pid_turn_set(138_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(34_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(-42_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(44_in, 127);
}

void blue_right_main(){
  mogo.set(false);
  chassis.pid_drive_set(-8.6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-92_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-0.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain(); 
  Intake_Conveyor(200);
  pros::delay(700);

  Intake_Conveyor(200);
  chassis.pid_drive_set(1.2_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(130_deg,TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35.3_in, 90); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1.5, 75);
  chassis.pid_wait_quick_chain();
  mogo.set(true); 
  pros::delay(550);
  Intake_Conveyor(200);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(25.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait();
  Intake_Conveyor(200);
  chassis.pid_turn_set(-92_deg, TURN_SPEED); 
  chassis.pid_wait();
  Intake_Conveyor(200);
  chassis.pid_drive_set(13.3_in, 60);
  chassis.pid_wait();
  Intake_Conveyor(200);
  pros::delay(500);
  chassis.pid_drive_set(-10.5_in, 78);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_turn_set(-74_deg, TURN_SPEED);  
  chassis.pid_wait();
  chassis.pid_drive_set(15.2_in, 65);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(32_in, 80);
  ladybrown.move_relative(2000, 150);
}

void red_left_main(){
  Intake_Conveyor(200);
  pros::delay(100);
  Intake_Conveyor(0);

  mogo.set(false);
  chassis.pid_drive_set(-8.6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(92_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-0.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain(); 
  Intake_Conveyor(200);
  pros::delay(700);

  Intake_Conveyor(200);
  chassis.pid_drive_set(1.2_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-130_deg,TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35.3_in, 80); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2.5, 65);
  chassis.pid_wait();
  mogo.set(true); 
  pros::delay(550);
  Intake_Conveyor(200);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(25.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait();
  pros::delay(500);
  Intake_Conveyor(200);
  chassis.pid_turn_set(92_deg, TURN_SPEED); 
  chassis.pid_wait();
  Intake_Conveyor(200);
  chassis.pid_drive_set(14_in, 60);
  chassis.pid_wait();
  pros::delay(400);
  Intake_Conveyor(200);
  chassis.pid_drive_set(-10.5_in, 78);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_turn_set(74_deg, TURN_SPEED);  
  chassis.pid_wait();
  Intake_Conveyor(200);
  chassis.pid_drive_set(15.4_in, 65);
  chassis.pid_wait();
  pros::delay(900);
  chassis.pid_turn_set(190_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(40_in, 127);
  ladybrown.move_relative(2000,120);
}

void blue_left_goal_rush()
{
  //rush to the mobile goal and take in the ring
  chassis.drive_angle_set(-30_deg);
  Intake_Conveyor(200);
  chassis.pid_drive_set(36_in, 110);
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
  chassis.pid_drive_set(-12_in, 75);
  chassis.pid_wait();
  mogo.set(true);
  Intake_Conveyor(200);
  pros::delay(100);
  chassis.pid_drive_set(6_in, 110);
  chassis.pid_wait();

  //release the mobile goal and grab the other one
  mogo.set(false);
  chassis.pid_turn_set(-97_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_drive_set(-13_in, 75);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(300);

  //turn and take in the preload ring
  chassis.pid_turn_set(145, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(20_in, 110);
  chassis.pid_wait();

  //go touch the hang
  chassis.pid_turn_set(45, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17.5_in, 110);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
}

void red_right_wall_stake(){
  mogo.set(false);
  chassis.pid_drive_set(-8.6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-0.7_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain(); 
  Intake_Conveyor(200);
  pros::delay(400);

  Intake_Conveyor(200);
  chassis.pid_drive_set(1.2_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(130_deg,TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35.3_in, 90); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-1.5, 75);
  chassis.pid_wait_quick_chain();
  mogo.set(true); 
  pros::delay(550);
  Intake_Conveyor(200);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(25.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait();
  Intake_Conveyor(200);
  pros::delay(200);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(45_in, DRIVE_SPEED_HIGH);
  ladybrown.move_relative(2000, 150);

}

void blue_left_wall_stake(){
  mogo.set(false);
  chassis.pid_drive_set(-8.6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(92_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-0.4_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain(); 
  Intake_Conveyor(200);
  pros::delay(700);

  Intake_Conveyor(200);
  chassis.pid_drive_set(1.2_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-130_deg,TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-35.3_in, 90); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-2.2, 70);
  chassis.pid_wait();
  mogo.set(true); 
  pros::delay(550);
  Intake_Conveyor(200);

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(25.5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait();
  pros::delay(500);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(45_in, DRIVE_SPEED_HIGH);
  ladybrown.move_relative(2000, 150);
}


void red_right_goal_rush_ver_2(){
  mogo.set(false);
  Intake_Conveyor(50);
  chassis.pid_drive_set(40_in, DRIVE_SPEED_HIGH-20);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  chassis.pid_drive_set(-4_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-48_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(9.5_in, DRIVE_SPEED_LOW-10);
  chassis.pid_wait();
  doinker_pneumatic.set(true);
  pros::delay(500); 
  chassis.pid_drive_set(-10.7_in, 38);
  chassis.pid_wait();
  doinker_pneumatic.set(false);
  pros::delay(150);
  chassis.pid_turn_set(153_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14.6_in, 60);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(500);
  Intake_Conveyor(200);
  pros::delay(300);
  Intake_Conveyor(0);
  chassis.pid_drive_set(13.8_in, DRIVE_SPEED_LOW-20);
  chassis.pid_wait();
  pros::delay(200);
  mogo.set(false);
  pros::delay(120);
  Intake_Conveyor(0);
  chassis.pid_turn_set(65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-19.7_in, 70);
  chassis.pid_wait();
  mogo.set(true);
  Intake_Conveyor(200);
  pros::delay(500);
  // gap xong stake 2
  chassis.pid_turn_set(-120_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(8_in, DRIVE_SPEED_HIGH);
  ladybrown.move_relative(2000, 150);
}

void blue_left_goal_rush_ver_2()
{
  //rush to the mobile goal and take in the ring
  chassis.drive_angle_set(-30_deg);
  Intake_Conveyor(50);
  chassis.pid_drive_set(36_in, 110);
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
  chassis.pid_drive_set(-12_in, 75);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(500);
  Intake_Conveyor(200);
  pros::delay(450);
  Intake_Conveyor(0);
  chassis.pid_drive_set(6_in, 110);
  chassis.pid_wait();

  //release the mobile goal and grab the other one
  mogo.set(false);
  chassis.pid_turn_set(-97_deg, TURN_SPEED, ez::cw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-13_in, 75);
  chassis.pid_wait();
  mogo.set(true);
  pros::delay(300);

  //turn and take in the preload ring
  chassis.pid_turn_set(145, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(20_in, 110);
  chassis.pid_wait();

  //go touch the hang
  chassis.pid_turn_set(45, TURN_SPEED, ez::ccw);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(17.5_in, 110);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
}

void red_right_ver_2(){
  mogo.set(true); //lift the piston
  chassis.pid_drive_set(-29_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  mogo.set(true); //piston down
  pros::delay(300);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-155_deg, TURN_SPEED); // old = 25 deg
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  chassis.pid_drive_set(3_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  pros::delay(300);
  chassis.pid_drive_set(12_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(125_deg, TURN_SPEED);  // old = 115 deg
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  chassis.pid_drive_set(15_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);
  ladybrown.move_relative(2000, 150);
  chassis.pid_drive_set(6_in, DRIVE_SPEED_LOW);
}

void red_right_ver_6(){
  chassis.pid_drive_set(-12_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  pros::delay(800);
  chassis.pid_turn_set(97_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-27_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  mogo.set(true); 
  chassis.pid_drive_set(2_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain(); 
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  pros::delay(250);
}

void red_right_ver_5(){
  //doinker = 10_in
  //3-4
  mogo.set(true); //lift the piston
  chassis.pid_drive_set(-25_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  mogo.set(true); //piston down
  pros::delay(300);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_turn_set(115_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  //mogo.set(true);
  chassis.pid_drive_set(30_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(62_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(true);
  pros::delay(200);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);
  pros::delay(300);
  chassis.pid_turn_set(115_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(15_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(150_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(10_in, DRIVE_SPEED_LOW);
  ladybrown.move_relative(2000, 150);
}


void red_right_ver_7(){
  //doinker = 10_in
  //3-4
  mogo.set(true); //lift the piston
  chassis.pid_drive_set(-26_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  // chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);

  // chassis.pid_wait();
  mogo.set(true); //piston down
  pros::delay(300);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);  
  chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(-6_in, DRIVE_SPEED_LOW);
  // chassis.pid_wait();
  chassis.pid_turn_set(-83_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(22_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  pros::delay(500);
  Intake_Conveyor(0);
  // pros::delay(300);

  chassis.pid_turn_set(78_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  chassis.pid_drive_set(41_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(true);
  pros::delay(200);
  
  chassis.pid_turn_set(105_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);
  pros::delay(300);
  chassis.pid_turn_set(125_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(50);
  chassis.pid_drive_set(20_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(0);

  chassis.pid_turn_set(-165_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-20_in, DRIVE_SPEED_LOW-15);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200); 
}

void red_right_ver_8(){
  //doinker = 10_in
  //3-4
  mogo.set(true); //lift the piston
  chassis.pid_drive_set(-25_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  // chassis.pid_drive_set(-5_in, DRIVE_SPEED_LOW);

  // chassis.pid_wait();
  mogo.set(true); //piston down
  pros::delay(300);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);  
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-6_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  pros::delay(300);
  chassis.pid_turn_set(115_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  mogo.set(true);
  pros::delay(100);
  chassis.pid_drive_set(30_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(62_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(19_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(true);
  pros::delay(200);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  doinker_pneumatic.set(false);
  pros::delay(300);
  chassis.pid_turn_set(115_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(50);
  chassis.pid_drive_set(15_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  Intake_Conveyor(200);
  chassis.pid_drive_set(-17_in, DRIVE_SPEED_LOW);
  chassis.pid_turn_set(25_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(24_in, DRIVE_SPEED_HIGH);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-65_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in, DRIVE_SPEED_LOW);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-155_deg, TURN_SPEED);
  Intake_Conveyor(200);
  pros::delay(800);
  Intake_Conveyor(0);
}

*/