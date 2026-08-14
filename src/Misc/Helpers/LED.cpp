#include "LED.h"

const uint8_t LED::kPins[4] = {2, 3, 4, 5};

namespace {
bool ledInit = false;
bool currentCompletedMode = false;
bool currentSearchingMode = false;
bool allBlinkOn = true;
unsigned long allBlinkLastToggleMs = 0;
uint8_t sequenceIndex = 0;
bool sequenceOnPhase = true;
unsigned long sequenceLastStepMs = 0;
uint8_t searchPhase = 0;
unsigned long searchLastToggleMs = 0;

void setAllLeds(bool on) {
	for (uint8_t pin : LED::kPins) {
		digitalWrite(pin, on ? HIGH : LOW);
	}
}

void resetSequenceState(unsigned long nowMs) {
	sequenceIndex = 0;
	sequenceOnPhase = true;
	sequenceLastStepMs = nowMs;
	setAllLeds(false);
	digitalWrite(LED::kPins[sequenceIndex], HIGH);
}

void resetBlinkState(unsigned long nowMs) {
	allBlinkOn = true;
	allBlinkLastToggleMs = nowMs;
	setAllLeds(true);
}

void resetSearchState(unsigned long nowMs) {
	searchPhase = 0;
	searchLastToggleMs = nowMs;
	setAllLeds(false);
	digitalWrite(LED::kPins[0], HIGH);
	digitalWrite(LED::kPins[3], HIGH);
}

void applySearchPhase(uint8_t phase) {
	setAllLeds(false);
	if (phase == 0) {
		digitalWrite(LED::kPins[0], HIGH);
		digitalWrite(LED::kPins[3], HIGH);
	} else if (phase == 2) {
		digitalWrite(LED::kPins[1], HIGH);
		digitalWrite(LED::kPins[2], HIGH);
	}
}
}  // namespace

void LED::init() {
	if (ledInit) {
		return;
	}

	for (uint8_t pin : kPins) {
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
	}
	ledInit = true;
}

void LED::update(bool running, bool searching, bool extinguished) {
	init();

	unsigned long nowMs = millis();
	if (!running && !extinguished) {
		setAllLeds(false);
		currentCompletedMode = false;
		currentSearchingMode = false;
		return;
	}

	if (extinguished) {
		if (!currentCompletedMode) {
			currentCompletedMode = true;
			currentSearchingMode = false;
			resetSequenceState(nowMs);
		}

		if (nowMs - sequenceLastStepMs < kSequenceStepMs) {
			return;
		}
		sequenceLastStepMs = nowMs;

		if (sequenceOnPhase) {
			digitalWrite(kPins[sequenceIndex], LOW);
			sequenceOnPhase = false;
		} else {
			sequenceIndex = (sequenceIndex + 1) % 4;
			digitalWrite(kPins[sequenceIndex], HIGH);
			sequenceOnPhase = true;
		}
		return;
	}

	if (searching) {
		if (!currentSearchingMode) {
			currentSearchingMode = true;
			currentCompletedMode = false;
			resetSearchState(nowMs);
		}

		if (nowMs - searchLastToggleMs < kSearchPhaseMs) {
			return;
		}
		searchLastToggleMs = nowMs;
		searchPhase = (searchPhase + 1) % 4;
		applySearchPhase(searchPhase);
		return;
	}

	if (currentCompletedMode) {
		currentCompletedMode = false;
		currentSearchingMode = false;
		resetBlinkState(nowMs);
	}

	if (currentSearchingMode) {
		currentSearchingMode = false;
		resetBlinkState(nowMs);
	}

	if (nowMs - allBlinkLastToggleMs < kAllBlinkMs) {
		return;
	}
	allBlinkLastToggleMs = nowMs;
	allBlinkOn = !allBlinkOn;
	setAllLeds(allBlinkOn);
}
