#include "my_custom_PID.hpp"

my_custom_PID::my_custom_PID (float kP, float kI, float kD)
{
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    integral = 0;
    prev_error = 0;
}

float my_custom_PID::update(float error)
{
    //calculate integral
    integral += error;

    //calculate derivative
    derivative = error - prev_error;
    prev_error = error;

    //return output
    return error*kP + integral*kI + derivative*kD;
}

void my_custom_PID::reset()
{
    integral = 0;
    prev_error = 0;
}

