#include <stdbool.h>
#include <math.h>

struct PID{

float target_speed;
float previous_speed;
float error_integral;

bool start;

float proportional_k, integral_k, derivative_k;
float voltage_limit, voltage_minimum;
};


// Init function for pid handler
void pid_init(struct PID *memory, float target_speed, float p, float i, float d, float voltage_limit, float voltage_minimum){
    memory->target_speed = target_speed;
    memory->previous_speed = 0;
    memory->error_integral = 0;
    memory->start = true;
    
    memory->proportional_k = fabs(p);
    memory->integral_k = fabs(i);
    memory->derivative_k = fabs(d);
    memory->voltage_limit = voltage_limit;
    memory->voltage_minimum = voltage_minimum;
}

// PID update function called at every timestep;
float pid_update(struct PID *memory, float current_speed, float dt){
    float voltage;

    // Proportional
    float error = memory->target_speed - current_speed;

    // Handling timer wrap arounds and micro changes
    if (dt <= 0) {
        voltage = error * memory->proportional_k;

        // Limit
        if (voltage > memory->voltage_limit) return memory->voltage_limit;
        if (voltage < memory->voltage_minimum) return memory->voltage_minimum;

        return voltage;
    }

    // Integral
    if (memory->integral_k > 0){
        memory->error_integral += error * dt;

        // Clamping integral to safeguard against slow starts and large overshoots
        if (memory->error_integral * memory->integral_k > memory->voltage_limit){
            memory->error_integral = memory->voltage_limit / (memory->integral_k);
        }

        if (memory->error_integral * memory->integral_k < memory->voltage_minimum){
            memory->error_integral = memory->voltage_minimum / (memory->integral_k);
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
    voltage = (error * memory->proportional_k) + 
    (memory->error_integral * memory->integral_k) - 
    (derivative_error * memory->derivative_k);

    if (voltage < memory->voltage_minimum) return memory->voltage_minimum;

    if (voltage > memory->voltage_limit) return memory-> voltage_limit;

    return voltage;
}