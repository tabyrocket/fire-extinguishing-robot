#include "Ultrasonic.h"

#define TRIGGER_PIN 48
#define ECHO_PIN 49

/* ENABLE / DISABLE SENSOR */

void Ultrasonic::enable()
{
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

void Ultrasonic::disable()
{
    pinMode(TRIGGER_PIN, INPUT);
    pinMode(ECHO_PIN, OUTPUT);
}

/* READ SENSOR */

void Ultrasonic::read()
// read distance from ultrasonic sensor - Note: This is a blocking call
{
    // reset pin   
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    // trigger pin to high for 10us
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 12000); // returns in us, timeout after about 2 m
    if (duration == 0) {
        distance = 1300;
        return;
    }

    distance = (duration / 2) * 0.34;  // distance in mm
}
