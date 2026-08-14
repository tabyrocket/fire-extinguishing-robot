#include <Arduino.h>
#include "Motors.h"

Motors::Motors()
{
    this->motors[0] = Motor( // Left front
        46, //Pin
        0 //Direction
        );

    this->motors[1] = Motor( // Right front
        51, //Pin
        1 //Direction
        );

    this->motors[2] = Motor( // Left rear
        47, //Pin
        0 //Direction
        );

    this->motors[3] = Motor( // Right rear
        50, //Pin
        1 //Direction
        );
}

/* ENABLE / DISABLE MOTORS */

void Motors::enable()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].enable();
    }
}

void Motors::disable()
{
    for(int i = 0; i < 4; i++)
    {
        motors[i].disable();
    }
}

/* SET MOTOR SPEEDS */

void Motors::set_speed(int motor, int speed)
// sets speed of given motor
{
    // Cap speed to 500
    speed = abs(speed) > 500 ? 500 * Helpers::sgn(speed) : speed;

    // Set motor speed
    motors[motor - 1].set_speed(speed);
}

int Motors::get_speed(int motor)
// get current speed of given motor
{
    return motors[motor - 1].get_speed();
}