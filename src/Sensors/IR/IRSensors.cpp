#include "IRSensors.h"


/* CONSTRUCTORS */

IRSensors::IRSensors(){
    this->sensors[0] = IRSensor( // FrontLeft
        A8, // Pin 
        74940, // Coefficient
        -1.079, // Exponent
        104, // Calibration Constant
        102 // Calibration Shift
        );

    this->sensors[1] = IRSensor( // Left
        A9, // Pin 
        26500.44, // Coefficient
        -1.04, // Exponent
        410, // Calibration Constant
        412 // Calibration Shift
        );

    this->sensors[2] = IRSensor( // Right
        A10, // Pin 
        26500.44, // Coefficient (GP2Y0A21YK0F)
        -1.04, // Exponent
        410, // Calibration Constant
        412 // Calibration Shift
        );

    this->sensors[3] = IRSensor( // FrontRight 
        A11, // Pin 
        74940, // Coefficient (GP2Y0A41SK0F)
        -1.079, // Exponent
        104, // Calibration Constant
        102 // Calibration Shift
        );
}

/* ENABLE / DISABLE SENSORS */

void IRSensors::enable(){
    for(int i = 0; i < 4; i++){
        this->sensors[i].enable();
    }
}

void IRSensors::disable(){
    for(int i = 0; i < 4; i++){
        this->sensors[i].disable();
    }
}

/* READ SENSORS */

void IRSensors::read(int sensor){
    if(sensor == -1){
        for(int i = 0; i < 4; i++){
            this->sensors[i].read();
        }
    } else {
        this->sensors[sensor].read();
    }
}
