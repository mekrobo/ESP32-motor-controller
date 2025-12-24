#include <Arduino.h>
#include "PID.h"

int pid_compute(pid_param *pid, float desired_value, float measured_value)
{   
    int error;
    int cmd_pwm;
    error = desired_value - measured_value;
    pid->integral_error += error;
    pid->integral_error = constrain(pid->integral_error,-pid->integral_max, pid->integral_max);
    if (desired_value == 0 && error == 0)
    {
        pid->integral_error =0;
    }
    cmd_pwm = (int)(desired_value * pid->kff + error * pid->kp + (error - pid->prev_error) * pid->kd + pid->integral_error * pid->ki);
    cmd_pwm = constrain(cmd_pwm,-pid->pwm_max,pid->pwm_max);
    pid->prev_error = error;
    return cmd_pwm;
}

/**
 * @brief Online pid tuning
 * 
 * @param pid 
 * @param kp 
 * @param ki 
 * @param kd 
 */
void update_PIDConstants(pid_param *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
