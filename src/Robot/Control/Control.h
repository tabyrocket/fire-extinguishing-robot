#ifndef Control_H
#define Control_H

#include <Arduino.h>

#include "../../Actuators/Fan/Fan.h"
#include "../../Actuators/FanServo/FanServo.h"
#include "../../Actuators/Motors/Motors.h"
#include "../../Sensors/Gyro/Gyro.h"
#include "../../Sensors/IR/IRSensors.h"
#include "../../Sensors/Phototransistor/Phototransistors.h"
#include "../../Sensors/Ultrasonic/Ultrasonic.h"

class Control {
 public:
  Control() {}
  Control(Motors* motors, Fan* fan, FanServo* fanServo, Gyro* gyro,
          IRSensors* irSensors, Ultrasonic* ultrasonic,
          Phototransistors* phototransistors);

  void enable();
  void disable();

  void read(int i);

  void moveDirection(double x, double y, double w, double speed);
  void moveDirectionCapX(double x, double y, double w);
  void setMotionLock(bool locked);
  bool isMotionLocked() const;

  void controlAngle(double angle);

  // fire direction control
        void aim(int sweepAngle = 75, bool leftToRight = true,
                                         bool allowWeakRetry = true);
  void aim2();
  void aim3();
  void finalAim();
  double correct();

  Motors* motors;
  Fan* fan;
  FanServo* fanServo;

  bool motionLocked = false;

  Gyro* gyro;
  IRSensors* irSensors;
  Ultrasonic* ultrasonic;
  Phototransistors* phototransistors;
};

#endif
