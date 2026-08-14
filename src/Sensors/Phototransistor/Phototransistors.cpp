#include "Phototransistors.h"


/* CONSTRUCTORS */

Phototransistors::Phototransistors() {
    this->sensors[0] = Phototransistor(
        A2 // Pin 
    );

    this->sensors[1] = Phototransistor( 
         A3 // Pin 
    );

    this->sensors[2] = Phototransistor( 
         A6 // Pin 
    );

    this->sensors[3] = Phototransistor( 
        A5 // Pin 
    );
}

/* ENABLE / DISABLE SENSORS */

void Phototransistors::enable() {
    for (int i = 0; i < 4; i++) {
        this->sensors[i].enable();
    }
}

void Phototransistors::disable() {
    for (int i = 0; i < 4; i++) {
        this->sensors[i].disable();
    }
}

/* READ SENSORS */

void Phototransistors::read(int sensor) {
    if (sensor == -1) {
        for (int i = 0; i < 4; i++) {
            this->sensors[i].read();
        }
    }
    else {
        this->sensors[sensor].read();
    }
}
