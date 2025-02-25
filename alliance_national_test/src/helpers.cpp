#include "main.h"

//rotation sensor
pros::Rotation rotation_sensor(5);

//vision sensor
pros::Vision vision_sensor(13);

// intake & conveyor
pros::MotorGroup intake_motor({-12, 11}, pros::v5::MotorGears::green);

void Intake_Conveyor (int intake_power) {
    intake_motor.move_velocity(intake_power);
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

// hang pneumatic
// true = hang
// false = normal
ez::Piston right_hang('B', false);
ez::Piston left_hang('E', false);
void Hang (bool check) {
    right_hang.button_toggle(check);
    left_hang.button_toggle(check);
}

// corner pneumatics
ez::Piston doinker_pneumatic('A', false);

void Doinker (bool check) {
    doinker_pneumatic.button_toggle(check);
}