#include "main.h"

//rotation
extern pros::Rotation rotation_sensor;

//vision
extern pros::Vision vision_sensor;

//intake & conveyor
extern pros::MotorGroup intake_motor;
void Intake_Conveyor(int intake_power);

//ladybrown
extern pros::Motor ladybrown;
void Ladybrown(int ladybrown_power);

//grab stake
extern ez::Piston mogo;
void Mogo (bool check);

// hang
extern ez::Piston right_hang;
void Right_hang (bool check);

extern ez::Piston left_hang;
void Left_hang (bool check);

// doinker
extern ez::Piston doinker_pneumatic;
void Doinker (bool check);

