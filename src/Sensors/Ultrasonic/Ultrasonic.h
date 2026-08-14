#ifndef Ultrasonic_H
#define Ultrasonic_H

#include <Arduino.h>

class Ultrasonic
{
    public:
        Ultrasonic(){}

        void enable();
        void disable();

        void read();

        double distance;
};

#endif