#ifndef FUZZY_H
#define FUZZY_H

#include <Arduino.h>
#include "../../../Misc/Helpers/Helpers.h"

struct FuzzySet {
    int size;
    float a[3];
    float b[3];
    float c[3];
    float d[3];
    float e[3];
};

struct FuzzyValue {
    float a;
    float b;
    float c;
    float d;
    float e;
};

struct FuzzySensors {
    FuzzyValue Front;
    FuzzyValue FrontLeft;
    FuzzyValue FrontRight;
    FuzzyValue Left;
    FuzzyValue Right;
    FuzzyValue Angle;
    FuzzyValue Correction;
};

struct FuzzyResult {
    float a;
    float b;
    float c;
    float d;
    float e;
    float none;
    
    FuzzyResult operator+ (const FuzzyResult fr){
        return {this->a + fr.a, this->b + fr.b, this->c + fr.c, this->d + fr.d, this->e + fr.e};
    }
    
    FuzzyResult operator* (const float m){
        return {this->a * m, this->b * m, this->c * m, this->d * m, this->e * m};
    }
};

struct FuzzyOutput {
    FuzzyResult x;
    FuzzyResult y;
    FuzzyResult w;
    
    FuzzyOutput operator+ (const FuzzyOutput fo){
        return {this->x + fo.x, this->y + fo.y, this->w + fo.w};
    }
    
    FuzzyOutput operator* (const float m){
        return {this->x * m, this->y * m, this->w * m};
    }

};

struct FuzzyCentroid {
    float centroid;
    float area;
};

class FuzzyLogic{
    public:
        static const FuzzySet Ultrasonic_FS;
        static const FuzzySet LeftFront_FS;
        static const FuzzySet RightFront_FS;
        static const FuzzySet Left_FS;
        static const FuzzySet Right_FS;
        static const FuzzySet Gyro_FS;
        static const FuzzySet Correction_FS;

        static const FuzzySet X_FS;
        static const FuzzySet Y_FS;
        static const FuzzySet W_FS;
        
        static float linz_fmf(float x, float p[2]);
        static float lins_fmf(float x, float p[2]);
        static float tri_fmf(float x, float p[3]);
        
        static FuzzyCentroid linz_dmf(float x, float p[2]);
        static FuzzyCentroid lins_dmf(float x, float p[2]);
        static FuzzyCentroid tri_dmf(float x, float p[3]);
        
        static FuzzyOutput aggregate(FuzzyOutput a, FuzzyOutput b);
        
        static FuzzyValue fuzzification(float x, FuzzySet fs);
        static float defuzzification(FuzzyResult fr, FuzzySet fs);
        
        static void test();
        
        static void control(
            // Inputs
            float ultrasonic, 
            float Left, 
            float Right, 
            float LeftFront, 
            float RightFront, 
            float gyro,
            float correction,
            // Outputs
            float* x, 
            float* y, 
            float* w,
            bool avoid = 1,
            int* rot_count = 0,
            bool gap = 0
        );
        
        static char getMax(FuzzyValue fv);
};

#endif