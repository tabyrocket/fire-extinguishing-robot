#ifndef Fan_H
#define Fan_H

#include <Arduino.h>

class Fan
{
public:
    Fan() {}

    void enable();
    void disable();
    void set_status(bool);

};

#endif