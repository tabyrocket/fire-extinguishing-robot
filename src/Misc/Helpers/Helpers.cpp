#include "Helpers.h"

HardwareSerial& Helpers::BTserial = Serial1;

void Helpers::serialBegin()
{
    BTserial.begin(115200);
}