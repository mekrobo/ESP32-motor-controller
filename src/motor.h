#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

const int ledChannelA1 = 0;
const int ledChannelB1 = 1;
const int ledChannelA2 = 2;
const int ledChannelB2 = 3;
const int resolution = 8;

enum channel {M1, M2};

struct rpm
{
    float motor1;
    float motor2;
};
struct current
{
    float motor1;
    float motor2;
};


class MotorController
{
    public:
        MotorController(channel motor_channel);
        void rotate(int speed);
        void current(float &current);
    private:
        channel motor_channel_;
        int8_t pin_current_;
        void init();
        uint8_t ledChannelA_;
        uint8_t ledChannelB_;
};

#endif