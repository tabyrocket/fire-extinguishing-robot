#include "Kalman.h"

Kalman::Kalman(float Q, float R, float P, float X)
{
    this->Q = Q;
    this->R = R;
    this->P = P;
    this->X = X;
}

double Kalman::getFilteredValue(double measurement)
{
    // TODO   
    
    return this->X;
}