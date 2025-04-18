#include "main.h"

int conveyor_direction = 1;
double hue;
double proximity;
bool ring_detected = false;
double conveyor_position;

void optical_task()
{
  optical_sensor.set_integration_time(10);
  while (true)
  {
    optical_sensor.disable_gesture();
    if (conveyor_motor.get_actual_velocity() > 0) optical_sensor.set_led_pwm(70);
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
