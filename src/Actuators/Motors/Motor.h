#ifndef Motor_H
#define Motor_H

#include <Arduino.h>
#include <Servo.h>

class Motor
{
    public:
        Motor(){}
        Motor(int, bool);

        void enable();
        void disable();

        void set_speed(int);
        int get_speed();

    private:
        int pin;
        Servo servo;
        int speed;
        bool direction;
};

#endif