#include "pid.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include<windows.h>

int main(){
    // PID constraints
    float target = 10;
    float kp, ki, kd;

    printf("Enter P gain: ");
    scanf(" %f", &kp);

    printf("Enter I gain:");
    scanf(" %f", &ki);
    
    printf("Enter D gain:");
    scanf(" %f", &kd);

    float maximum = 30; 
    float minimum = 0;

    // Input and output
    float voltage = 0;
    float speed = 0;

    double dt = 0.1;

    // Initialising PID
    struct PID handler;
    pid_init(&handler, target, kp, ki, kd, maximum, minimum);
    
    // Update loop
    int count = 0;
    int iteration = 0;

    while (count < 10 && iteration < 500){
        iteration++;
        Sleep(100);

        // Logic updates
        speed = voltage * 0.5;
        voltage = pid_update(&handler, speed, dt);

        printf("%i. Speed: %.2f  Voltage: %.2f\n", iteration, speed, voltage);

        // Checking stability
        if (fabsf(speed - target) < 0.1f){
            count++;
        } else count = 0;
    }

    return 0;    
}