#ifndef Phototransistors_H
#define Phototransistors_H

#include <Arduino.h>

#include "./Phototransistor.h"

class Phototransistors
{
public:
    Phototransistors();

    void enable();
    void disable();

    void read(int sensor = -1);

    Phototransistor sensors[4];
};

#endif