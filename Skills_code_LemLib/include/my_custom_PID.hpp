#pragma once

class my_custom_PID
{
    private:
    double kP;
    double kI;
    double kD;
    
    double integral;
    double derivative;
    double prev_error;

    public:
    my_custom_PID (double, double, double);
    double update(double);
    void reset();
};