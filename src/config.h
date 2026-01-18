#ifndef CONFIG_H
#define CONFIG_H

// Motor Channel
#define Channel_1 M1
#define Channel_2 M2

// Motor PINs PWM
#define MOTOR_DRIVER
#define MOTOR1_PWMA 13
#define MOTOR1_PWMB 12
#define MOTOR2_PWMA 14
#define MOTOR2_PWMB 27

// Motor PINs current
#define MOTOR1_C 25
#define MOTOR2_C 34

#define PWM_MAX 255 //(2^PWMBITS - 1)
#define PWM_FREQ 7000 // PWM frequency in Hz
//Motor Encoder pin
#define MOTOR1_ENC_A 2
#define MOTOR1_ENC_B 4
#define MOTOR2_ENC_A 16
#define MOTOR2_ENC_B 17

// Motor Specification
#define MAX_RPM 300 // motor  maximum RPM at output shaft
#define MAX_ACC 20000 // motor Acceleration at base motor          
#define COUNTS_PER_REV_1 2709.2 // wheel encoder's no of ticks per rev motor shaft
#define COUNTS_PER_REV_2  9048 // wheel encoder's no of ticks per rev motor shaft

// Timer Seetings
#define SENSOR_PERIOD_MS 40 // Sensor read period in ms (>=20 ms)
#define CONTROL_PERIOD_MS 40 // Control loop period in ms

// PID parameters for RPM control
#define Kff_1 0.7245 // RPM to PWM mapping for motor 1
#define Kfr_1 4.128 // PWM for drive friction of motor 1
#define KP_1 2 // P constant for motor 1
#define KI_1 0.2 // I constant for motor 1
#define KD_1 0 // D constant for motor 1

#define Kff_2  2.405 // RPM to PWM mapping for motor 2 0.7187
#define Kfr_2 4.0236 // PWM for drive friction of motor 2
#define KP_2 2 // P constant
#define KI_2 0.2 // I constant
#define KD_2 0 // D constant

#endif // CONFIG_H