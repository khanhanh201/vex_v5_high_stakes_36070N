#include "main.h"

bool run_conveyor = false;
void conveyor_auto(bool run)
{
    run_conveyor = run;
}

//rotation sensor
pros::Rotation rotation_sensor(5);

//optical
pros::Optical optical_sensor(13);

// intake
pros::Motor intake_motor(11, pros::v5::MotorGears::green);
void Intake (int intake_power) {
    intake_motor.move_velocity(intake_power);
}

// conveyor
pros::Motor conveyor_motor(-12, pros::v5::MotorGears::blue);
void Conveyor (int conveyor_power) {
    conveyor_motor.move_velocity(conveyor_power);
}


// ladybrown 
pros::Motor ladybrown (6, pros::v5::MotorGears::green);

void Ladybrown (int ladybrown_power) {
    ladybrown.move_velocity(ladybrown_power);  
}

// mogo 
// true = grasp
// false = release
ez::Piston mogo('C', false);
void Mogo (bool check) {
    mogo.button_toggle(check);
}


// doinker
// true = down
// false = up
ez::Piston left_doinker_pneumatic('H', false);
void Left_doinker (bool check) {
    left_doinker_pneumatic.button_toggle(check);
}

// true = down
// false = up
ez::Piston right_doinker_pneumatic('A', false);
void Right_doinker (bool check) {
    right_doinker_pneumatic.button_toggle(check);
}