#pragma once

extern int conveyor_direction;
extern double hue;
extern double proximity;
extern bool ring_detected;

void optical_task();
extern pros::Task Optical_task(pros::task_fn_t);

void intake_auto_task();
extern pros::Task Intake_Auto_task(pros::task_fn_t);