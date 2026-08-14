#include <Arduino.h>
#include "Fan.h"

#define PIN 9 
/* ENABLE / DISABLE FAN */

void Fan::enable()
{
        pinMode(PIN, OUTPUT);
}

void Fan::disable()
{
    pinMode(PIN, INPUT);
}

void Fan::set_status(bool status)
{
    if (status){
        digitalWrite(PIN, HIGH);
    }
    else {
        digitalWrite(PIN, LOW);
    }
}