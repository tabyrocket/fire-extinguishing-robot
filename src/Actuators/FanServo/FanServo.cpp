#include <Arduino.h>
#include "FanServo.h"

#define PIN 10
#define LOGICAL_MIN_ANGLE -75
#define LOGICAL_MAX_ANGLE 75
#define SERVO_LEFT_ANGLE 150
#define SERVO_CENTER_ANGLE 90
#define SERVO_RIGHT_ANGLE 30
#define SERVO_STEP_DELAY_MS 10
/* CONSTRUCTOR */

FanServo::FanServo()
{
    this->angle = 0;
}

/* ENABLE / DISABLE MOTOR */

void FanServo::enable()
{
    pinMode(PIN, OUTPUT);
    this->servo.attach(PIN);
    this->set_angle(0);
}

void FanServo::disable()
{
    this->servo.detach();
    pinMode(PIN, INPUT);
}

/* SET / READ MOTOR SPEED */
// max angles are -60 and 60
void FanServo::set_angle(int angle)
{
    this->set_angle(angle, SERVO_STEP_DELAY_MS);
}

void FanServo::set_angle(int angle, int stepDelayMs)
{
    angle = constrain(angle, LOGICAL_MIN_ANGLE, LOGICAL_MAX_ANGLE);

    int step = angle > this->angle ? 1 : -1;

    while (this->angle != angle) {
        this->angle += step;

        int servoAngle;
        if (this->angle < 0) {
            servoAngle = map(this->angle, LOGICAL_MIN_ANGLE, 0, SERVO_LEFT_ANGLE, SERVO_CENTER_ANGLE);
        } else {
            servoAngle = map(this->angle, 0, LOGICAL_MAX_ANGLE, SERVO_CENTER_ANGLE, SERVO_RIGHT_ANGLE);
        }

        this->servo.write(servoAngle);
        delay(stepDelayMs);
    }
}

int FanServo::get_angle()
{
    return this->angle;
}
