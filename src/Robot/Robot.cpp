#include "Robot.h"

Robot::Robot() {
    this->motors = new Motors();
    this->fan = new Fan();
    this->fanServo = new FanServo();

    this->gyro = new Gyro();
    this->irSensors = new IRSensors();
    this->ultrasonic = new Ultrasonic();
    this->phototransistors = new Phototransistors();

    this->control = new Control(
        this->motors, 
        this->fan,
        this->fanServo,
        this->gyro, 
        this->irSensors, 
        this->ultrasonic, 
        this->phototransistors
        );
    this->finiteStateMachine = new FSM(this->control);
}

void Robot::enable() {
    finiteStateMachine->start();
    control->enable();
}

void Robot::disable() {
    finiteStateMachine->stop();
    control->disable();
}