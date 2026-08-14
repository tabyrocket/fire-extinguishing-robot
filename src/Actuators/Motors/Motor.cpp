#include <Arduino.h>
#include "Motor.h"

/* CONSTRUCTOR */

Motor::Motor(int pin, bool direction)
{
    this->pin = pin;
    this->speed = 0;
    this->direction = direction;
}

/* ENABLE / DISABLE MOTOR */

void Motor::enable()
{
    if(pin!=0){
        pinMode(this->pin, OUTPUT);
        this->servo.attach(this->pin);
    }
}

void Motor::disable()
{
    this->servo.detach();
    pinMode(this->pin, INPUT);
}

/* SET / READ MOTOR SPEED */

void Motor::set_speed(int speed)
{
    this->speed = (direction) ? -speed : speed;
    this->servo.writeMicroseconds(1500+this->speed);
}

int Motor::get_speed()
{
    return this->speed;
}