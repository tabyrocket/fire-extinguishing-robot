#ifndef LED_H
#define LED_H

#include <Arduino.h>

class LED {
public:
	static const uint8_t kPins[4];
	static constexpr unsigned long kAllBlinkMs = 500UL;
	static constexpr unsigned long kSearchPhaseMs = 100UL;
	static constexpr unsigned long kSequenceStepMs = 50UL;

	static void init();
	static void update(bool running, bool searching, bool extinguished);
};

#endif