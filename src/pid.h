#ifndef PID_H
#define PID_H

#include <stdbool.h>
#include <math.h>

// PID Handler struct
struct PID{
    float target;
    float previous_measurement;
    float error_integral;
    float proportional_k, integral_k, derivative_k;
    float output_maximum, output_minimum;
    bool start;
};


void pid_init(struct PID *memory, float target, float p, float i, float d, float output_maximum, float output_minimum);
float pid_update(struct PID *memory, float current_measurement, double dt);
void target_change(struct PID *memory, float new_target);

#endif