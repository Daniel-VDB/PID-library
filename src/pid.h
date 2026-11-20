#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <math.h>

struct PID{

float target;
float previous_speed;
float error_integral;

bool start;

float proportional_k, integral_k, derivative_k;
float output_maximum, output_minimum;
};


void pid_init(struct PID *memory, float target, float p, float i, float d, float output_maximum, float output_minimum);
float pid_update(struct PID *memory, float current_speed, float dt);
void target_change(struct PID *memory, float new_target);

#endif