#ifndef Motors_H
#define Motors_H

#include <Arduino.h>

#include "./Motor.h"
#include "../../Misc/Helpers/Helpers.h"

class Motors
{
    public:
        Motors();

        void enable();
        void disable();

        void set_speed(int motor, int speed);
        int get_speed(int motor);

    private:
        Motor motors[4];
};

#endif