#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

///
// Constants
///
void default_constants() {
  liftPID.exit_condition_set(100, 3, 500, 7, 500, 500);
  chassis.pid_heading_constants_set(11, 0, 20);
  chassis.pid_drive_constants_set(20, 0, 100);
  chassis.pid_turn_constants_set(3, 0.05, 20, 15);
  chassis.pid_swing_constants_set(6, 0, 65);

  chassis.pid_turn_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(80_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(80_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.slew_drive_constants_set(7_in, 80);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
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
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}
void blue_left(){
  lift_pneumatic.set(true);
  mogo.set(false) // lift the piston -> nhả mogo
  chassis.pid_drive_set(-32_in, DRIVE_SPEED);
  chassis.pid_wait();
  // move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  mogo.set(true);
  pros:: delay(300);
  Conveyor(200);
  pros::delay(800);
  Conveyor(0);
  Intake(200);
  chassis.pid_wait();
  chassis.pid_turn_set(65,TURN_SPEED);
  chassis.pid_wait();
  Conveyor(200);
  chassis.pid_drive_set(35,TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-35_in, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(180, TURN_SPEED);
  Intake(0);
  Conveyor(0);
  chassis.pid_wait();
  ladybrown.move_relative(2000,150);

}
void skill(){
  // move forward 12 inches                    
  chassis.pid_drive_set(9_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  // turn to 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 8 inches
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4_in, 80, true); 
  chassis.pid_wait();
  //grab stake and move backward 8 inches
  mogo.set(true);
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  //turn to 0 degrees 
  chassis.pid_turn_set(0_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 18 inches and start intake and conveyor                           
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true); 
  Intake(200); 
  Conveyor(200);
  chassis.pid_wait();
  // turn to 90 degrees clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 40 inches
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true);//old 48
  chassis.pid_wait();
  // move backward 12 inches
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);  
  chassis.pid_wait_quick_chain();
  // turn to 180 degress clockwise 
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move forward 66 inches 
  chassis.pid_drive_set(66_in, 80, true); 
  chassis.pid_wait();
  // move backward 24 inches
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true); //old -15_in
  chassis.pid_wait();
  // turn to 45 degrees counter-clockwise and stop intake and conveyor
  Intake(0);
  Conveyor(0);
  chassis.pid_turn_set(-45_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 30 inches and release stake 
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); 
  mogo.set(false);
  chassis.pid_wait_quick_chain();
  // turn to 60 degrees counter-clockwise
  chassis.pid_turn_set(-60_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 36 inches 
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);// old 36
  chassis.pid_wait_quick_chain();
  // turn to 90 degrees clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // move backward 48 inches 
  chassis.pid_drive_set(-52_in, DRIVE_SPEED, true);// old -48
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, 90, true);
  mogo.set(true);
  chassis.pid_wait();
  // turn to 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED);// old -90_deg
  chassis.pid_wait_quick_chain();
  // move forward 72 inches and start intake and conveyor
  Intake(200);
  Conveyor(200);
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  // move backward 12 inches 
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true); // old -12_in
  chassis.pid_wait_quick_chain();
  // turn to 20 degrees clockwise
chassis.pid_turn_set(20_deg, TURN_SPEED);//old 20_deg
  chassis.pid_wait_quick_chain();
  // move forward 72 inches 
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true); //old 72_in
  chassis.pid_wait(); // -> pause 
  // move backward 90 inches and release stake
  chassis.pid_drive_set(-90_in, DRIVE_SPEED - 20, true); 
  chassis.pid_wait_quick_chain(); // Allows the program to continue running other tasks while checking if the PID controller has finished its task.
  mogo.set(false);
  // move forward 140 inches and stop conveyor
  // -----end of section 1: the red half of playing field-------------------------
  // when moving forward, (grab mogo) -> (ring intake) -> have to turn 180 degrees to grab mogo
  chassis.pid_drive_set(140_in, DRIVE_SPEED, true); 
  chassis.pid_wait(); // stop motor
  Conveyor(0);
  // turn to 135 degrees counter-clockwise
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move backward 40 inches 
  //
  chassis.pid_drive_set(-40_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, 90, true);
  mogo.set(true);
  chassis.pid_wait(); 
  // turn to 75 degrees counter-clockwise
  chassis.pid_turn_set(-75_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move backward 48 inches and release stake
  chassis.pid_drive_set(-48_in, DRIVE_SPEED, true); 
  Conveyor(200);
  chassis.pid_wait();
  mogo.set(false); // THẢ MOGO VÀO CORNER
  // turn to 135 degrees clockwise
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move forward 70 inches and start conveyor
  chassis.pid_drive_set(70_in, DRIVE_SPEED, true); 
  Conveyor(200);
  chassis.pid_wait();
  Conveyor(0);
  // turn to 135 degrees counter-clockwise
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move backward 30 inches 
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, 90, true);
  mogo.set(true);
  chassis.pid_wait();
  // move backward 5 inches 
  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  // turn to 135 degrees clockwise
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move forward 36 inches 
  chassis.pid_drive_set(36_in, DRIVE_SPEED, true); 
  Conveyor(200);
  chassis.pid_wait();
  // turn to 135 degrees clockwise
  Intake(0);
  Conveyor(0);
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move backward 72 inches 
  chassis.pid_drive_set(-72_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
}
//---------------eND Of ver 1-------------------------
// CODE ĐÃ TUNE GÓC BUỔI SÁNG
void skill()
{
  // move forward 12 inches                    
  chassis.pid_drive_set(9_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  // turn to 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 8 inches
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-4_in, 80, true); 
  chassis.pid_wait();
  //grab stake and move backward 8 inches
  mogo.set(true);
  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); 
  chassis.pid_wait_quick_chain();
  //turn to 0 degrees 
  chassis.pid_turn_set(0_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 18 inches and start intake and conveyor                           
  chassis.pid_drive_set(22_in, DRIVE_SPEED, true); 
  Intake(200); 
  Conveyor(200);
  chassis.pid_wait();
  // turn to 90 degrees clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 40 inches
  chassis.pid_drive_set(40_in, DRIVE_SPEED, true); //old 48
  chassis.pid_wait();
  // move backward 12 inches
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true);  
  chassis.pid_wait_quick_chain();
  // turn to 180 degress clockwise 
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  // move forward 66 inches 
  chassis.pid_drive_set(66_in, 80, true); 
  chassis.pid_wait();
  // move backward 24 inches
  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true); //old -15_in
  chassis.pid_wait();
  // turn to 45 degrees counter-clockwise and stop intake and conveyor
  Intake(0);
  Conveyor(0);
  chassis.pid_turn_set(-45_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move backward 30 inches and release stake 
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); ///////// change to 20 inch
  mogo.set(false);
  chassis.pid_wait_quick_chain();
  // turn to 60 degrees counter-clockwise
  chassis.pid_turn_set(-60_deg, TURN_SPEED); 
  chassis.pid_wait_quick_chain();
  // move forward 36 inches 
  chassis.pid_drive_set(20_in, DRIVE_SPEED, true);// old 36      ///////////// change to 21 inch
  chassis.pid_wait_quick_chain();
  // turn to 90 degrees clockwise
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  // move backward 48 inches 
  chassis.pid_drive_set(-50_in, DRIVE_SPEED, true);// old -48
  chassis.pid_wait_quick_chain();
  //move backward 5 inches and grasp the mobile goal
  chassis.pid_drive_set(-5_in, 90, true);
  mogo.set(true);
  chassis.pid_wait();
  // turn to 90 degrees counter-clockwise
  chassis.pid_turn_set(-90_deg, TURN_SPEED);// old -90_deg
  chassis.pid_wait_quick_chain();
  // move forward 72 inches and start intake and conveyor
  Intake(200);
  Conveyor(200);
  chassis.pid_drive_set(72_in, DRIVE_SPEED, true); 
  chassis.pid_wait();
  // move backward 12 inches
  chassis.pid_drive_set(-3_in, DRIVE_SPEED, true); // old -12_in
  chassis.pid_wait_quick_chain();
  // turn to 20 degrees clockwise
  chassis.pid_turn_set(20_deg, TURN_SPEED);//old 20_deg
  chassis.pid_wait_quick_chain();
  // move forward 72 inches 
  chassis.pid_drive_set(48_in, DRIVE_SPEED, true); //old 72_in
  chassis.pid_wait();
  // move backward 90 inches and release stake
  chassis.pid_drive_set(-90_in, DRIVE_SPEED - 20, true);    /////////////// change to 86
  chassis.pid_wait_quick_chain();
  mogo.set(false);


  // move forward inches and stop conveyor
  Conveyor(0);
  Intake(0);
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(28_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(120_in, DRIVE_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_set(-85, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(45_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-120_in, DRIVE_SPEED);
  chassis.pid_wait();
}