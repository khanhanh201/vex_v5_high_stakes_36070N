#pragma once

//ladybrown macros

extern double ladybrown_hold_angle;
extern double ladybrown_up_angle;
extern double ladybrown_down_angle;

extern my_custom_PID ladybrown_PID;
extern double rotation_error;
extern int rotation_count;
extern bool move_done;

void ladybrown_move_PID(double, double, int);
void ladybrown_task();
//extern pros::Task Ladybrown_Task(pros::task_fn_t);
