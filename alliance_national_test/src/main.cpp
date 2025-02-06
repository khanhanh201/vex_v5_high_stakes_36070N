#include "main.h"

//ladybrown macros
#define ladybrown_angle 21
#define ladybrown_deadband 0.6
#define ladybrown_time_count 4

//Vision
#define color_detect 1 //1: blue, 2:red



// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-1, -2, 3},     // Left Chassis Ports (negative port will reverse it!)
    {-8, 9,  10},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    4.125,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    280);   // Wheel RPM = cartridge * (motor gear / wheel gear)



my_custom_PID ladybrown_PID(6, 0, 0.1);
double rotation_error;
int rotation_count;
bool manual_ladybrown = true;

void ladybrown_move_PID(double target)
{
  pros::lcd::set_text(4, "not done");
	bool move_done = false;
  rotation_count = 0;
	ladybrown_PID.reset();

	while (!move_done)
	{
		rotation_error = target - ((double)(rotation_sensor.get_position())/100);
		ladybrown.move_velocity(ladybrown_PID.update(rotation_error));

    if(master.get_digital(DIGITAL_UP) || master.get_digital(DIGITAL_DOWN)) move_done = true;

		if (std::abs(rotation_error) < ladybrown_deadband) rotation_count++;
    if (rotation_count >= ladybrown_time_count) move_done = true;

		pros::delay(ez::util::DELAY_TIME);
	}
  pros::lcd::set_text(4, "done");
  ladybrown.brake();
	ladybrown_PID.reset();
  manual_ladybrown = true;
}

void ladybrown_task()
{
  ladybrown.set_brake_mode(MOTOR_BRAKE_BRAKE);
  while (true)
  {
    if (!manual_ladybrown)
    {
      ladybrown_move_PID(ladybrown_angle);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Ladybrown_Task(ladybrown_task);


int direction = 1;
void vision_task()
{
  pros::delay(ez::util::DELAY_TIME);
  pros::vision_signature_s_t blue_signature = pros::Vision::signature_from_utility(1, -3863, -3307, -3585, 5689, 8119, 6904, 4.200, 0);
	pros::vision_signature_s_t red_signature = pros::Vision::signature_from_utility(2, 8629, 12603, 10616, -2185, -1329, -1757, 5.500, 0);
	vision_sensor.set_signature(1, &blue_signature);
	vision_sensor.set_signature(2, &red_signature);
	pros::vision_object_s_t largest_object;
  
  while (true)
  {
    largest_object = vision_sensor.get_by_sig(0, color_detect); //blue_signature

		if (vision_sensor.get_object_count() > 0 && largest_object.width >= 250 && largest_object.height >= 190 && intake_motor.get_actual_velocity() > 1)
		{
			direction = direction * (-1);
			pros::delay(120);
			direction = direction * (-1);
			pros::delay(500);
		}

		pros::delay(ez::util::DELAY_TIME);
  }
  
}
pros::Task my_Vision_Task(vision_task);

void display_task()
{
  while (true)
  {
    pros::lcd::set_text(3, std::to_string((double)(rotation_sensor.get_position())/100));
    pros::lcd::set_text(5, std::to_string(manual_ladybrown));
    if (color_detect == 1) pros::lcd::set_text(6, "color detected: blue");
    if (color_detect == 2) pros::lcd::set_text(6, "color detected: red");
    pros::lcd::set_text(7, std::to_string(intake_motor.get_actual_velocity()));
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Display_Task(display_task);


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

  ladybrown.tare_position();
  ladybrown.set_encoder_units(MOTOR_ENCODER_DEGREES);
  rotation_sensor.reset_position();
  rotation_sensor.set_reversed(true);

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}


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
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  negative_red();
}



void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  ladybrown.set_brake_mode(MOTOR_BRAKE_BRAKE);

  while (true) {
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade

    // intake & conveyor
    Intake_Conveyor((master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2))*200);

    // ladybrown

    if(master.get_digital(DIGITAL_LEFT)) manual_ladybrown = false;
    if(master.get_digital(DIGITAL_UP) || master.get_digital(DIGITAL_DOWN)) manual_ladybrown = true;

    if (manual_ladybrown) 
    {
      if (((double)(rotation_sensor.get_position())/100) > 150) Ladybrown(master.get_digital(DIGITAL_DOWN)*(-200));
      else
      {
        Ladybrown((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN))*200*direction);
      }
    }

    // mogo
    Mogo(master.get_digital(DIGITAL_X));

    // lift intake
    Lift(master.get_digital(DIGITAL_B));

    // doinker pneumatics
    Doinker(master.get_digital(DIGITAL_A));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
