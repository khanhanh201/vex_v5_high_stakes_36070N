#include "main.h"

//intake
extern pros::MotorGroup intake_motor;
void Intake(int intake_power);

// conveyer
extern pros::MotorGroup conveyer_motor;
void Conveyer (int conveyer_power);

//ladybrown
extern pros::MotorGroup ladybrown;
void Ladybrown(int ladybrown_power);

//grab stake
extern ez::Piston mogo;
void Mogo (bool check);

// lift intake
extern ez::Piston lift_pneumatic;
void Lift (bool check);

// corner
extern ez::Piston corner_pneumatic;
void Corner (bool check);