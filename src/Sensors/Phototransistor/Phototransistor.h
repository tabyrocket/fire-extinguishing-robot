#ifndef Phototransistor_H
#define Phototransistor_H

#include <Arduino.h>

class Phototransistor {
public:
    Phototransistor() {}
    Phototransistor(int pin);

    void enable();
    void disable();

    void read();

    double value;

private:
    int pin;
};

#endif