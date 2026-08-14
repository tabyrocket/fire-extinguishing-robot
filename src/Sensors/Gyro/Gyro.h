#ifndef Gyro_H
#define Gyro_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "../../Misc/Helpers/Helpers.h"

class Gyro
// keeps track of the angle turned by the gyro
// positive angle is clockwise
{
    public:
        Gyro();

        void enable();
        void disable();

        void calibrate();
        void update();
        void reset();

        double currentAngle;  // angle state value
        double angularVelocity;

    private:
        Adafruit_BNO08x bno;
        bool enabled;
        bool hasYaw;
        double yawAccumRad;
        double prevYawRad;
        double lastYawRad;

        void setReports();
};

#endif