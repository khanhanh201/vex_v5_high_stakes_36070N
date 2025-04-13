#include "main.h"

//rotation
extern pros::Rotation rotation_sensor;

//optical
extern pros::Optical optical_sensor;

//intake
extern pros::Motor intake_motor;
void Intake(int intake_power);

//conveyor
extern pros::Motor conveyor_motor;
void Conveyor(int conveyor_power);

//ladybrown
extern pros::Motor ladybrown;
void Ladybrown(int ladybrown_power);

//grab stake
extern ez::Piston mogo;
void Mogo (bool check);

// lift intake
extern ez::Piston left_doinker_pneumatic;
void Left_doinker (bool check);

// doinker
extern ez::Piston right_doinker_pneumatic;
void Right_doinker (bool check);

