#include "main.h"

//ladybrown macros
#define ladybrown_hold_angle 35
#define ladybrown_down_angle 5
#define ladybrown_up_angle 120
#define ladybrown_deadband 0.5
#define ladybrown_time_count 10


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

    7,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    400);   // Wheel RPM = cartridge * (motor gear / wheel gear)



my_custom_PID ladybrown_PID(4, 0, 12);
double rotation_error;
int rotation_count;
bool manual_ladybrown = true;
bool move_done = false;

void ladybrown_move_PID(double target)
{
  pros::lcd::set_text(4, "not done");
	move_done = false;
  manual_ladybrown = false;
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
  ladybrown_PID.reset();

  while (true)
  {
    if (master.get_digital(DIGITAL_LEFT))
    {
      ladybrown_move_PID(ladybrown_hold_angle);
    }
    
    if (master.get_digital(DIGITAL_RIGHT))
    {
      ladybrown_move_PID(ladybrown_up_angle);
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Ladybrown_Task(ladybrown_task);



int8_t conveyor_direction = 1;
double hue;
double proximity;
bool ring_detected = false;
void optical_task()
{
  double conveyor_position;
  optical_sensor.set_integration_time(10);
  while (true)
  {
    optical_sensor.disable_gesture();
    if (conveyor_motor.get_actual_velocity() > 0) optical_sensor.set_led_pwm(100);
    else optical_sensor.set_led_pwm(0);
  
    if (ring_detected == false)
    {
      hue = optical_sensor.get_hue();
      proximity = optical_sensor.get_proximity();
      
      if (hue > 220 && hue < 240 && conveyor_motor.get_actual_velocity() > 0)
      {
        master.rumble(".");
        conveyor_position = conveyor_motor.get_position();
        ring_detected = true;
      }
    }
    
    if (ring_detected == true && conveyor_motor.get_position() > conveyor_position + 100)
    {
      ring_detected = false;
      conveyor_direction = -1;
      pros::delay(150);
      conveyor_direction = 1;
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Optical_Task(optical_task);


bool run_conveyor = false;
void intake_auto_task()
{
  while (true)
  {
    while (pros::competition::is_autonomous)
    {
      if (run_conveyor) conveyor_motor.move_velocity(600*conveyor_direction);
      pros::delay(ez::util::DELAY_TIME);
    }
    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task Intake_Auto_Task(intake_auto_task);



void conveyor_auto(bool run)
{
  run_conveyor = run;
}



/*
int direction = 1;
bool enable_sort = true;
string color_detect = "blue";

void vision_task()
{
  pros::vision_signature_s_t blue_signature = pros::Vision::signature_from_utility(1, -4975, -4167, -4570, 9391, 11823, 10606, 3.500, 0);
	pros::vision_signature_s_t red_signature = pros::Vision::signature_from_utility(2, 11981, 15151, 13566, -2557, -1625, -2092, 3.000, 0);
	vision_sensor.set_signature(1, &blue_signature);
	vision_sensor.set_signature(2, &red_signature);
	pros::vision_object_s_t largest_object;
  
  if (color_detect == "blue") //blue
  {
    while (enable_sort)
    {
      largest_object = vision_sensor.get_by_sig(0, 1); //blue_signature
      vision_sensor.clear_led();

      pros::lcd::set_text(1, std::to_string(largest_object.width));
      pros::lcd::set_text(2, std::to_string(largest_object.height));

      //width:260, height 200
      if (vision_sensor.get_object_count() > 0 && largest_object.width > 230 && largest_object.height > 180 && intake_motor.get_actual_velocity() > 1)
      {
        pros::lcd::set_text(7, "blue ring seen");
        master.rumble("-");
        pros::delay(70);
        direction = -1;
        pros::delay(200);
        direction = 1;
        pros::delay(500);
      }
      else pros::lcd::set_text(7, "blue ring not seen");

      pros::delay(ez::util::DELAY_TIME);
    }
  }

  if (color_detect == "red") //red
  {
    while (enable_sort)
    {
      largest_object = vision_sensor.get_by_sig(0, 2); //red_signature
      vision_sensor.clear_led();

      pros::lcd::set_text(1, std::to_string(largest_object.width));
      pros::lcd::set_text(2, std::to_string(largest_object.height));

      //width:316, height 212
      if (vision_sensor.get_object_count() > 0 && largest_object.width > 230 && largest_object.height > 180 && intake_motor.get_actual_velocity() > 1)
      {
        pros::lcd::set_text(7, "red ring seen");
        pros::delay(70);
        master.rumble("-");
        direction = -1;
        pros::delay(170);
        direction = 1;
        pros::delay(500);
      }
      else pros::lcd::set_text(7, "red ring not seen");

      pros::delay(ez::util::DELAY_TIME);
    }
  }
  pros::delay(ez::util::DELAY_TIME);
}
pros::Task my_Vision_Task(vision_task);
*/


void display_task()
{
  while (true)
  {
    pros::lcd::set_text(3, "Ladybrown angle: " + std::to_string((double)(rotation_sensor.get_position())/100));
    pros::lcd::set_text(4, "Hue: " + std::to_string(hue));
    pros::lcd::set_text(5, "Proximity: " + std::to_string(proximity));

    //log the ladybrown's position
    if (!manual_ladybrown) std::cout <<((double)(rotation_sensor.get_position())/100) <<",\n";
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
  intake_motor.tare_position();
  mogo.set(false);

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

  //red_left_main();
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
    if(master.get_digital(DIGITAL_LEFT) || master.get_digital(DIGITAL_RIGHT)) manual_ladybrown = false;
    if(master.get_digital(DIGITAL_UP) || master.get_digital(DIGITAL_DOWN)) manual_ladybrown = true;

    if (manual_ladybrown) Ladybrown((master.get_digital(DIGITAL_UP) - master.get_digital(DIGITAL_DOWN))*200);

    // mogo
    Mogo(master.get_digital(DIGITAL_X));

    // doinker 
    Left_doinker(master.get_digital(DIGITAL_A));
    Right_doinker(master.get_digital(DIGITAL_Y));

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
