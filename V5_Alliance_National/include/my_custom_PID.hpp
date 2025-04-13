#pragma once

class my_custom_PID
{
    private:
    float kP;
    float kI;
    float kD;
    
    float integral;
    float derivative;
    float prev_error;

    public:
    my_custom_PID (float, float, float);
    float update(float);
    void reset();
};