#include <Arduino.h>
#include "PID.h"

int pid_compute(pid_param *pid, float desired_value, float measured_value)
{   
    int error;
    int cmd_pwm_for;
    int cmd_pwm;
    int signOfdes = constrain(desired_value, -1, 1);
    error = desired_value - measured_value;
    error = constrain(error,-10,10);
    pid->integral_error += error;
    pid->integral_error = constrain(pid->integral_error,-pid->integral_max, pid->integral_max);
    cmd_pwm_for = (int) (signOfdes*pid->fr + desired_value * pid->kff);
    cmd_pwm = (int)( error * pid->kp + (error - pid->prev_error) * pid->kd + pid->integral_error * pid->ki);
    pid->prev_error = error;
    if (abs(desired_value) <= 0.1)
    {
        pid->integral_error = 0;
        return 0;
    }
    if (abs(measured_value) <= 5)
    {
        constrain(cmd_pwm_for,-pid->pwm_max,pid->pwm_max);
        return cmd_pwm_for;
    }
    cmd_pwm += cmd_pwm_for;
    cmd_pwm = constrain(cmd_pwm,-pid->pwm_max,pid->pwm_max);
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
