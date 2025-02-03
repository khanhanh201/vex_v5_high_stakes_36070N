#include "main.h"

//ladybrown macros
#define ladybrown_angle 17.5
#define ladybrown_deadband 0.1
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



my_custom_PID ladybrown_PID(6, 0, 1);
double rotation_error;
int rotation_count;
bool manual_ladybrown = true;

void ladybrown_move_PID(float target)
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

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Negative red", negative_red},
      {"test ace", test_ACE},
      {"Interference\n\nAfter driving forward, robot performs differently if interfered or not", interfered_example},
      {"Simple Odom\n\nThis is the same as the drive example, but it uses odom instead!", odom_drive_example},
      {"Pure Pursuit\n\nGo to (0, 30) and pass through (6, 10) on the way.  Come back to (0, 0)", odom_pure_pursuit_example},
      {"Pure Pursuit Wait Until\n\nGo to (24, 24) but start running an intake once the robot passes (12, 24)", odom_pure_pursuit_wait_until_example},
      {"Boomerang\n\nGo to (0, 24, 45) then come back to (0, 0, 0)", odom_boomerang_example},
      {"Boomerang Pure Pursuit\n\nGo to (0, 24, 45) on the way to (24, 24) then come back to (0, 0, 0)", odom_boomerang_injected_pure_pursuit_example},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", odom_boomerang_injected_pure_pursuit_example},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  ladybrown.tare_position();
  ladybrown.set_encoder_units(MOTOR_ENCODER_DEGREES);
  rotation_sensor.reset_position();
  rotation_sensor.set_reversed(true);

  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}



/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  ladybrown.set_brake_mode(MOTOR_BRAKE_BRAKE);

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    //ez_template_extras();

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
