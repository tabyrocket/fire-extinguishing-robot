#ifndef IRSensor_H
#define IRSensor_H

#include <Arduino.h>

class IRSensor{
    public:
        IRSensor(){}
        IRSensor(int pin, double coefficient, double exponent, int calibrationConst, int calibrationShift);

        void enable();
        void disable();

        void read();
        
        double distance;

    private:
        int pin;
        double coefficient;
        double exponent;
        int calibrationConst;
        int calibrationShift;
};

#endif