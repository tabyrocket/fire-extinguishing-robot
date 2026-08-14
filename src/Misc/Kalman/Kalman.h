#ifndef Kalman_H
#define Kalman_H

#include <Arduino.h>

class Kalman
{
    public:
        Kalman(){}
        Kalman(float Q, float R, float P, float X);
        
        double getFilteredValue(double measurement);
        
    private:
        float Q; // Process noise variance
        float R; // Measurement noise variance
        float P; // Estimation error variance
        float X; // Value
        float K; // Kalman gain
};

#endif