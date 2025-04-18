#pragma once

extern bool run_conveyor;
void conveyor_auto(bool run);

//rotation
extern pros::Rotation rotation_sensor;

//optical
extern pros::Optical optical_sensor;

//intake
extern pros::Motor intake_motor;
void Intake(int);

//conveyor
extern pros::Motor conveyor_motor;
void Conveyor(int);

//ladybrown
extern pros::Motor ladybrown;
void Ladybrown(int);

//grab stake
extern ez::Piston mogo;
void Mogo (bool);

// lift intake
extern ez::Piston left_doinker_pneumatic;
void Left_doinker (bool);

// doinker
extern ez::Piston right_doinker_pneumatic;
void Right_doinker (bool);
