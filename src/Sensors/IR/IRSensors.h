#ifndef IRSensors_H
#define IRSensors_H

#include <Arduino.h>

#include "./IRSensor.h"

class IRSensors
{
    public:
        IRSensors();

        void enable();
        void disable();

        void read(int sensor = -1);

        IRSensor sensors[4];
};

#endif