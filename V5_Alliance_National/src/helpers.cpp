#include "main.h"

//rotation sensor
pros::Rotation rotation_sensor(5);

// intake & conveyor
pros::MotorGroup intake_motor({-12, 11}, pros::v5::MotorGears::green);

void Intake_Conveyor (int intake_power) {
    intake_motor.move_velocity(intake_power);
}


// ladybrown 
pros::Motor ladybrown (-6, pros::v5::MotorGears::green);

void Ladybrown (int ladybrown_power) {
    ladybrown.move_velocity(ladybrown_power);  
}

ez::PID liftPID{0.45, 0.05, 0, 0, "Lift"};

void liftWait(){
    while (liftPID.exit_condition(ladybrown, true) == ez::RUNNING) {
        pros::delay(ez::util::DELAY_TIME);
    }
}

void liftTask(){
    pros::delay(2000);
    bool controlled_by_pid = false;
    while (true) {
        if (controlled_by_pid)
        {
            Ladybrown(liftPID.compute(ladybrown.get_position()));
            pros::delay(ez::util::DELAY_TIME);
        }

        if (master.get_digital_new_press(DIGITAL_LEFT)) controlled_by_pid = true;
        if (master.get_digital_new_press(DIGITAL_UP) || master.get_digital(DIGITAL_DOWN)) controlled_by_pid = false;
    }
}

// mogo 
// true = grasp
// false = release   
ez::Piston mogo('C', false);

void Mogo (bool check) {
    mogo.button_toggle(check);
}

// lift intake
// true = up
// false = down
ez::Piston lift_pneumatic('B', false);

void Lift (bool check) {
    lift_pneumatic.button_toggle(check);
}

// corner pneumatics
ez::Piston doinker_pneumatic('A', false);

void Doinker (bool check) {
    doinker_pneumatic.button_toggle(check);
}