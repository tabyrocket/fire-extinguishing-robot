
#include "Gyro.h"

Gyro::Gyro()
{
    this->angularVelocity = 0.0;
    this->currentAngle = 0.0;
    this->enabled = false;
    this->hasYaw = false;
    this->yawAccumRad = 0.0;
    this->prevYawRad = 0.0;
    this->lastYawRad = 0.0;
}

/* ENABLE / DISABLE GYRO */

void Gyro::enable()
{
    if (enabled) {
        return;
    }

    if (!bno.begin_I2C()) {
        Helpers::telePrint("Gyro", "BNO08x init failed");
        return;
    }

    setReports();
    enabled = true;
    reset();
}

void Gyro::disable()
{
    enabled = false;
}

/* CALIBRATE / UPDATE / RESET GYRO */

void Gyro::calibrate()
{
    reset();
}

void Gyro::update()
{
    if (!enabled) {
        return;
    }

    sh2_SensorValue_t sensorValue;
    while (bno.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR || sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            double qw = sensorValue.un.rotationVector.real;
            double qx = sensorValue.un.rotationVector.i;
            double qy = sensorValue.un.rotationVector.j;
            double qz = sensorValue.un.rotationVector.k;

            double yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            lastYawRad = yawRad;

            if (!hasYaw) {
                prevYawRad = yawRad;
                yawAccumRad = 0.0;
                hasYaw = true;
            } else {
                double delta = yawRad - prevYawRad;
                if (delta > PI) {
                    delta -= 2.0 * PI;
                } else if (delta < -PI) {
                    delta += 2.0 * PI;
                }
                yawAccumRad += delta;
                prevYawRad = yawRad;
            }

            currentAngle = yawAccumRad * 180.0 / PI;
        } else if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
            angularVelocity = sensorValue.un.gyroscope.z * 180.0 / PI;
        }
    }
}

void Gyro::reset() 
// reset gyro angle
{
    currentAngle = 0;
    angularVelocity = 0;
    yawAccumRad = 0.0;
    prevYawRad = lastYawRad;
    hasYaw = false;
}

void Gyro::setReports()
{
    bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
}