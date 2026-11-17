struct PID{

float target_speed;
float previous_speed;
float error_integral;

float proportional_k, integral_k, derivative_k;
};

void pid_init(struct PID *memory, float target_speed, float p, float i, float d){
    memory->target_speed = target_speed;
    memory->previous_speed = 0;
    memory->error_integral = 0;

    memory->proportional_k = p;
    memory->integral_k = i;
    memory->derivative_k = d;
}

float pid_update(struct PID *memory, float current_speed, float dt){
    // Proportional
    float error = memory->target_speed - current_speed;

    // Integral
    memory->error_integral += error * dt;

    // Derivative (on measurement, not error)
    float derivative_error = (current_speed - memory->previous_speed) / dt;

    // Update memory
    memory->previous_speed = current_speed;

    return (error * memory->proportional_k) +
    (memory->error_integral * memory->integral_k) -
    (derivative_error * memory->derivative_k);
}