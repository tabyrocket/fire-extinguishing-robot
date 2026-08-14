#ifndef Helpers_H
#define Helpers_H

#include <Arduino.h>

class Helpers
{
public:

    static HardwareSerial& BTserial;
    
    static void serialBegin();
    
    template <typename T>
    static T sgn(T const x)
    {
        // Function to return the sign of x
        return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
    }


    template <typename T>
    static T movingAverage(T sensor, T prevEstimate, double Q = 3, double P = 0.3)
    {
        double K = P / (P + Q);
        return prevEstimate + K * (sensor - prevEstimate);
    }
    

    template <typename T>
    static void serialPrint(T val, int param = 0)
    {
        // Print to Serial Monitor
        BTserial.print(val);
        if (param == 1) {
            BTserial.println();
        } else if (param == 2) {
            BTserial.print(",");
        }
    }
    
    template <typename T, typename S>
    static void telePrint(S sensorName, T sensorValue)
    {
        // Print to Bluetooth
        BTserial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
        BTserial.print(sensorName);
        BTserial.print(": ");
        BTserial.print(sensorValue);
        BTserial.println();
    }
};

#endif