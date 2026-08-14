#ifndef FanServo_H
#define FanServo_H

#include <Arduino.h>
#include <Servo.h>

class FanServo
{
public:
    FanServo();

    void enable();
    void disable();

    void set_angle(int);
    void set_angle(int, int);
    int get_angle();

private:
    Servo servo;
    int angle;
};

#endif
