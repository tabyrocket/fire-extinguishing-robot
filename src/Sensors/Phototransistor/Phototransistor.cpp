#include "Phototransistor.h"
#include "../../Misc/Helpers/Helpers.h"

/* CONSTRUCTOR */
Phototransistor::Phototransistor(int pin) {
    this->pin = pin;
}

/* ENABLE / DISABLE SENSOR*/

void Phototransistor::enable() {
    pinMode(this->pin, INPUT);
}

void Phototransistor::disable() {
    pinMode(this->pin, OUTPUT);
}

/* READ SENSOR */

void Phototransistor::read() {
    int raw = analogRead(this->pin);
    this->value = 1023 - raw;
}