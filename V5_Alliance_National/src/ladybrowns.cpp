#include "main.h"

//ladybrown macros
double ladybrown_hold_angle = 24.5;
double ladybrown_up_angle = 153;
double ladybrown_down_angle = 5;

my_custom_PID ladybrown_PID(4, 0, 12);
double rotation_error;
int rotation_count;
bool move_done = true;

void ladybrown_move_PID(double target, double deadband, int time_count)
{
  move_done = false;
  rotation_count = 0;
	ladybrown_PID.reset();

	while (!move_done)
	{
		rotation_error = target - ((double)(rotation_sensor.get_position())/100);
		ladybrown.move_velocity(ladybrown_PID.update(rotation_error));

    if (master.get_digital(DIGITAL_UP) || master.get_digital(DIGITAL_DOWN)) move_done = true;
    if (master.get_digital(DIGITAL_RIGHT) && target == ladybrown_hold_angle) move_done = true;
    if (master.get_digital(DIGITAL_LEFT) && (target == ladybrown_up_angle || target == ladybrown_down_angle)) move_done = true;

		if (std::abs(rotation_error) < deadband) rotation_count++;
    if (rotation_count >= time_count) move_done = true;

		pros::delay(ez::util::DELAY_TIME);
  } 
  ladybrown.brake();
	ladybrown_PID.reset();
}

void ladybrown_task()
{
  ladybrown.set_brake_mode(MOTOR_BRAKE_BRAKE);
  ladybrown_PID.reset();

  while (true)
  {
    if (master.get_digital(DIGITAL_LEFT))
    {
      ladybrown_move_PID(ladybrown_hold_angle, 0.4, 3);
      master.rumble(".");
    }
    if (master.get_digital(DIGITAL_RIGHT))
    {
      ladybrown_move_PID(ladybrown_up_angle, 3, 1);
      ladybrown_move_PID(ladybrown_down_angle, 3, 1);
      master.rumble(".");
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}

//pros::Task Ladybrown_Task(ladybrown_task);