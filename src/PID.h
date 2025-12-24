#ifndef PID_H
#define PID_H

struct pid_param {
    float kff; // feeforward gain (RPM to PWM map)
    float fr; // friction compensation
    float kp;
    float ki;
    float kd;
    int pwm_max; //pwm_max driver
    float integral_max;
    float integral_error;
    float prev_error;
};
int pid_compute(pid_param *pid, float desired_value, float measured_value);
void update_PIDConstants(pid_param *pid, float kp, float ki, float kd);
#endif