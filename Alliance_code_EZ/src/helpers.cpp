#include "main.h"

// intake
pros::MotorGroup intake_motor({19, -20}, pros::v5::MotorGears::green);

void Intake (int intake_power) {
    intake_motor.move_velocity(intake_power);
}

// conveyer
pros::MotorGroup conveyer_motor({16, 15}, pros::v5::MotorGears::green);

void Conveyer (int conveyer_power) {
    conveyer_motor.move_velocity(conveyer_power);
}

// ladybrown
pros::MotorGroup ladybrown ({11, -12}, pros::v5::MotorGears::green);

void Ladybrown (int ladybrown_power) {
    ladybrown.move_velocity(ladybrown_power);
    
}

// mogo    
ez::Piston mogo('C', true);

void Mogo (bool check) {
    mogo.button_toggle(check);
}

// lift intake
ez::Piston lift_pneumatic('B', true);

void Lift (bool check) {
    lift_pneumatic.button_toggle(check);
}

// corner pneumatics
ez::Piston corner_pneumatic('A', false);

void Corner (bool check) {
    corner_pneumatic.button_toggle(check);
}