#include "IRSensor.h"

/* CONSTRUCTOR */
IRSensor::IRSensor(int pin, double coefficient, double exponent, int calibrationConst, int calibrationShift){
    this->pin = pin;
    this->coefficient = coefficient;
    this->exponent = exponent;
    this->calibrationConst = calibrationConst;
    this->calibrationShift = calibrationShift;
    this->distance = 0;
}

/* ENABLE / DISABLE SENSOR*/

void IRSensor::enable(){
    pinMode(this->pin, INPUT);
}
 
void IRSensor::disable(){
    pinMode(this->pin, OUTPUT);
}

/* READ SENSOR */

void IRSensor::read(){
    // int adjustedADC = analogRead(this->pin) + (this->calibrationConst - this->calibrationShift);
    // this->distance = this->coefficient * pow(adjustedADC, this->exponent);
    this->distance = analogRead(this->pin);
}