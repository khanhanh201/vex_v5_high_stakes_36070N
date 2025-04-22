#include "main.h"


/**
 * before each match:
 * - check auton
 * - check color sorted
 * - check ladybrown position
 */


// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, -2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-8, 9,  10},  // Right Chassis Ports (negative port will reverse it!)

    4,      // IMU Port
    3.375,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    400);   // Wheel RPM = cartridge * (motor gear / wheel gear)


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(2.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(2.1, 4);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  chassis.pid_tuner_full_enable(true);
  ladybrown.tare_position();
  ladybrown.set_encoder_units(MOTOR_ENCODER_DEGREES);
  rotation_sensor.reset_position();
  rotation_sensor.set_reversed(true);
  intake_motor.tare_position();
  // mogo.set(false);
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}
pros::Task Ladybrown_Task(ladybrown_task);

void display_task()
{
  while (true)
  {
    pros::lcd::set_text(2, "Ladybrown current angle: " + std::to_string((double)(rotation_sensor.get_position())/100));
    pros::lcd::set_text(3, "Ladybrown target angle: " + std::to_string(ladybrown_hold_angle));
    pros::lcd::set_text(4, "Hue: " + std::to_string(hue));
    pros::lcd::set_text(5, "Proximity: " + std::to_string(proximity));
    pros::lcd::set_text(6, "move_done: " + std::to_string(move_done));
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Display_Task(display_task);



void disabled() {
  // . . .
}


void competition_initialize() {
  // . . .
}


void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistence
  mogo.set(false);
  // positive_red();
  negative_red();
}



void opcontrol() {  
  // This is preference to what you like to drive on

  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  ladybrown.set_brake_mode(MOTOR_BRAKE_BRAKE);

  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    // intake
    Intake((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2))*200);

    //conveyor
    Conveyor((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2))*600*conveyor_direction);

    // ladybrown
    Ladybrown((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN))*200);

    // mogo
    Mogo(master.get_digital(DIGITAL_X));

    // doinker 
    Left_doinker(master.get_digital(DIGITAL_Y));
    Right_doinker(master.get_digital(DIGITAL_A));

    if (master.get_digital_new_press(DIGITAL_B)){
      chassis.pid_tuner_toggle();
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
