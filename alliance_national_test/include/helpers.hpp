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

// lift intake
extern ez::Piston lift_pneumatic;
void Lift (bool check);

// doinker
extern ez::Piston doinker_pneumatic;
void Doinker (bool check);

