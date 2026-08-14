#ifndef FSM_H
#define FSM_H

#include <Arduino.h>
#include <math.h>

#include "../Control/Control.h"
#include "../Control/Fuzzy/Fuzzy.h"
#include "../../Misc/Helpers/Helpers.h"

class FSM {
    public:
        FSM(){}
        FSM(Control* control);

        void start();
        void stop();
        void update();

        void setState(int state);
        
    // private:
        Control* control;
        
        float filteredIR0 = 0;
        float filteredIR1 = 0;
        float filteredIR2 = 0;
        float filteredIR3 = 0;
        float filteredUltra = 0;
        float filteredPT0 = 0;
        float filteredPT1 = 0;
        float filteredPT2 = 0;
        float filteredPT3 = 0;
        float filteredGyro = 0;

        enum {STOPPED, RUNNING};

        bool status;
        int currentState;
        long i;
        long enterTime;
        bool state2AimLeftToRight = true;
        unsigned long state3LastFireSeenMs = 0;
        unsigned long state4AlignStartMs = 0;
        int state3FrontEscapeCount = 0;
        float state3FrontEscapeY = 0.0f;
        int rot_countV = 0;
        int* rot_count = &rot_countV;
        
        int num_fires;
        
        float xV = 0;
        float yV = 0;
        float wV = 0;
        
        float *x = &xV;
        float *y = &yV; 
        float *w = &wV;
        
        bool avoid;
        
        void runState();
        bool updateStallDetection(float gyroAngle);
        void resetStallDetection();

        /* STATE FUNCTIONS */
        void state1_initialisation();
        void state2_search();
        void state3_move();
        void state4_extinguish();
        void state5_reverse();
        void state6_stalled();

        unsigned long stallLastSampleMs = 0;
        unsigned long stallStableStartMs = 0;
        float stallLastGyroAngle = 0.0f;
        unsigned long state6StartMs = 0;
        float wallBypassDir = 0.0f;
};

#endif
