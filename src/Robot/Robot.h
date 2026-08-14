#ifndef Robot_H
#define Robot_H

#include <Arduino.h>

#include "./FSM/FSM.h"
#include "./Control/Control.h"
#include "../Actuators/Fan/Fan.h"
#include "../Actuators/Motors/Motors.h"
#include "../Actuators/FanServo/FanServo.h"
#include "../Sensors/Gyro/Gyro.h"
#include "../Sensors/IR/IRSensors.h"
#include "../Sensors/Ultrasonic/Ultrasonic.h"
#include "../Sensors/Phototransistor/Phototransistors.h"

class Robot {
    public:
        Robot();

        void enable();
        void disable();

        FSM* finiteStateMachine;
        Control* control;
        
        Motors* motors;
        Fan* fan;
        FanServo* fanServo;

        Gyro* gyro;
        IRSensors* irSensors;
        Ultrasonic* ultrasonic;
        Phototransistors* phototransistors;
};

#endif