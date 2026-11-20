#include "pid.h"

// Init function for pid handler
void pid_init(struct PID *memory, float target, float p, float i, float d, float output_maximum, float output_minimum){
    memory->target = target;
    memory->previous_speed = 0;
    memory->error_integral = 0;
    memory->start = true;
    
    memory->proportional_k = fabs(p);
    memory->integral_k = fabs(i);
    memory->derivative_k = fabs(d);
    memory->output_maximum = output_maximum;
    memory->output_minimum = output_minimum;
}

// PID update function called at every timestep;
float pid_update(struct PID *memory, float current_speed, float dt){
    float output;

    // Proportional
    float error = memory->target - current_speed;

    // Handling timer wrap arounds and micro changes
    if (dt <= 0) {
        output = error * memory->proportional_k;

        // maximum
        if (output > memory->output_maximum) return memory->output_maximum;
        if (output < memory->output_minimum) return memory->output_minimum;

        return output;
    }

    // Integral
    if (memory->integral_k > 0){
        memory->error_integral += error * dt;

        // Clamping integral to safeguard against slow starts and large overshoots
        if (memory->error_integral * memory->integral_k > memory->output_maximum){
            memory->error_integral = memory->output_maximum / (memory->integral_k);
        }

        if (memory->error_integral * memory->integral_k < memory->output_minimum){
            memory->error_integral = memory->output_minimum / (memory->integral_k);
        }
    }
    
    // Derivative
    float derivative_error;
    
    // Prevent derivatives on startup from creating large inflated values
    if (memory->start) {
        memory->start = false;
        derivative_error = 0;
    } else {
        derivative_error = (current_speed - memory->previous_speed) / dt;
    }
    
    // Update memory
    memory->previous_speed = current_speed;
    
    // Determine safe return value
    output = (error * memory->proportional_k) + 
    (memory->error_integral * memory->integral_k) - 
    (derivative_error * memory->derivative_k);

    if (output < memory->output_minimum) return memory->output_minimum;

    if (output > memory->output_maximum) return memory-> output_maximum;

    return output;
}

void target_change(struct PID *memory, float new_target){
    memory->target = new_target;
    memory->error_integral = 0;
}