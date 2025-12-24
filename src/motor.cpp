#include "motor.h"
#include "config.h"

MotorController::MotorController(channel motor_channel):
    motor_channel_(motor_channel)
    
{   
    init();
}

void MotorController::init()
{
    if (motor_channel_ == M1) {
        ledChannelA_ = ledChannelA1;
        ledChannelB_ = ledChannelB1;
        pin_current_ = MOTOR1_C;
        pinMode(MOTOR1_PWMA,OUTPUT);
        pinMode(MOTOR1_PWMB,OUTPUT);
        digitalWrite(MOTOR1_PWMA,LOW);
        digitalWrite(MOTOR1_PWMB,LOW);
        ledcSetup(ledChannelA1, PWM_FREQ, resolution);
        ledcSetup(ledChannelB1, PWM_FREQ, resolution);
        // attach the channel to the GPIO to be controlled
        ledcAttachPin(MOTOR1_PWMA, ledChannelA_);
        ledcAttachPin(MOTOR1_PWMB, ledChannelB_);

    } else if (motor_channel_ == M2) {
        ledChannelA_ = ledChannelA2;
        ledChannelB_ = ledChannelB2;
        pin_current_ = MOTOR2_C;
        pinMode(MOTOR2_PWMA,OUTPUT);
        pinMode(MOTOR2_PWMB,OUTPUT);
        digitalWrite(MOTOR2_PWMA,LOW);
        digitalWrite(MOTOR2_PWMB,LOW);
        ledcSetup(ledChannelA2, PWM_FREQ, resolution);
        ledcSetup(ledChannelB2, PWM_FREQ, resolution);
        // attach the channel to the GPIO to be controlled
        ledcAttachPin(MOTOR2_PWMA, ledChannelA_);
        ledcAttachPin(MOTOR2_PWMB, ledChannelB_);

    }
}

void MotorController::rotate(int speed)
{
    if (speed > 0) {
        ledcWrite(ledChannelA_, speed);
        ledcWrite(ledChannelB_, 0);
    } else if (speed < 0) {
        ledcWrite(ledChannelA_, 0);
        ledcWrite(ledChannelB_, -speed);
    } else {
        ledcWrite(ledChannelA_, 0);
        ledcWrite(ledChannelB_, 0);
    }
}

void MotorController::current(float &current)
{
    current = ((float)analogRead(pin_current_)) * (3.3* 1.7483 / 4096); // Assuming 12-bit ADC and 3.3V reference
}
    