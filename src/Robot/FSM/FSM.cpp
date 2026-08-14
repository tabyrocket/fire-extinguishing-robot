#include "FSM.h"

#include "../../Misc/Helpers/LED.h"

FSM::FSM(Control* control) {
  this->control = control;

  status = STOPPED;
  currentState = 1;
  num_fires = 2;

  this->x = new float();
  this->y = new float();
  this->w = new float();
}

void FSM::setState(int state) { currentState = state; }

void FSM::start() {
  status = RUNNING;
  num_fires = 2;
}

void FSM::stop() { status = STOPPED; }

void FSM::resetStallDetection() {
  stallLastSampleMs = 0;
  stallStableStartMs = 0;
  stallLastGyroAngle = 0.0f;
}

bool FSM::updateStallDetection(float gyroAngle) {
  unsigned long nowMs = millis();
  const unsigned long samplePeriodMs = 100UL;
  const unsigned long stallWindowMs = 1000UL;
  const float gyroVariationThreshold = 0.5f;

  if (stallLastSampleMs == 0) {
    stallLastSampleMs = nowMs;
    stallStableStartMs = nowMs;
    stallLastGyroAngle = gyroAngle;
    return false;
  }

  if ((nowMs - stallLastSampleMs) < samplePeriodMs) {
    return false;
  }

  float gyroVariation = fabs(gyroAngle - stallLastGyroAngle);
  stallLastSampleMs = nowMs;
  stallLastGyroAngle = gyroAngle;

  if (gyroVariation < gyroVariationThreshold) {
    if (stallStableStartMs == 0) {
      stallStableStartMs = nowMs;
    }
    return (nowMs - stallStableStartMs) >= stallWindowMs;
  }

  stallStableStartMs = nowMs;
  return false;
}

void FSM::update() {
  switch (status) {
    case STOPPED:
      break;
    case RUNNING:
      runState();
      break;
  }

  LED::update(status == RUNNING, status == RUNNING && currentState == 2,
              num_fires == 0);
}

void FSM::runState() {
  // Run the current state
  switch (currentState) {
    case 1:
      state1_initialisation();
      break;
    case 2:
      state2_search();
      break;
    case 3:
      state3_move();
      break;
    case 4:
      state4_extinguish();
      break;
    case 5:
      state5_reverse();
      break;
    case 6:
      state6_stalled();
      break;
  }
}

void FSM::state1_initialisation() {
  // initialise
  control->enable();

  control->gyro->calibrate();
  control->gyro->reset();

  filteredIR0 = 0;
  filteredIR1 = 0;
  filteredIR2 = 0;
  filteredIR3 = 0;
  filteredUltra = 100;
  filteredPT0 = 100;
  filteredPT1 = 100;
  filteredPT2 = 100;
  filteredPT3 = 100;
  filteredGyro = 0;
  i = 0;
  avoid = 1;
  enterTime = 0;
  state3LastFireSeenMs = 0;
  state4AlignStartMs = 0;
  state6StartMs = 0;
  wallBypassDir = 0.0f;
  resetStallDetection();

  if (i % 10 == 0) {
    Helpers::telePrint("State", "1_init");
    Helpers::telePrint("StateId", currentState);
    Helpers::telePrint("i", i);
    Helpers::telePrint("Avoid", avoid);
  }

  // Update PTs
  int j = 0;
  float lastPT1 = 0;
  float lastPT2 = 0;
  while (j < 100) {
    control->phototransistors->read(-1);
    float ptLimit = 1024;
    float PT1 = isnan(control->phototransistors->sensors[1].value)
                    ? ptLimit
                    : control->phototransistors->sensors[1].value;
    float PT2 = isnan(control->phototransistors->sensors[2].value)
                    ? ptLimit
                    : control->phototransistors->sensors[2].value;

    lastPT1 = PT1;
    lastPT2 = PT2;

    filteredPT1 = Helpers::movingAverage(PT1, filteredPT1, 5, 1);
    filteredPT2 = Helpers::movingAverage(PT2, filteredPT2, 5, 1);
    j++;
    delay(1);
  }

  if (i % 10 == 0) {
    Helpers::telePrint("PTsum", lastPT1 + lastPT2);
    Helpers::telePrint("FPTsum", filteredPT1 + filteredPT2);
  }

  // Next state
  currentState = 2;
}

void FSM::state2_search() {
  if (i % 10 == 0) {
    Helpers::telePrint("State", "2_search");
    Helpers::telePrint("StateId", currentState);
    Helpers::telePrint("i", i);
  }
  // search for fire - rotate towards fire
  control->aim(75, state2AimLeftToRight, num_fires > 1);
  control->gyro->reset();
  resetStallDetection();

  delay(100);
  state3LastFireSeenMs = millis();

  currentState = 3;
}

void FSM::state3_move() {
  // Fuzzy Obstacle Avoidance
  //
  control->fanServo->set_angle(0, 2);

  // Needed vars
  float IR0, IR1, IR2, IR3, ultra, PT0, PT1, PT2, PT3, gyro;

  // Read Sensors
  control->read(i);

  float irLimit = 1024;
  float usLimit = 1300;
  float ptLimit = 1024;

  IR0 = isnan(control->irSensors->sensors[0].distance) ||
                (control->irSensors->sensors[0].distance > irLimit)
            ? -irLimit
            : -control->irSensors->sensors[0].distance;
  IR1 = isnan(control->irSensors->sensors[1].distance) ||
                (control->irSensors->sensors[1].distance > irLimit)
            ? -irLimit
            : -control->irSensors->sensors[1].distance;
  IR2 = isnan(control->irSensors->sensors[2].distance) ||
                (control->irSensors->sensors[2].distance > irLimit)
            ? -irLimit
            : -control->irSensors->sensors[2].distance;
  IR3 = isnan(control->irSensors->sensors[3].distance) ||
                (control->irSensors->sensors[3].distance > irLimit)
            ? -irLimit
            : -control->irSensors->sensors[3].distance;
  ultra = isnan(control->ultrasonic->distance) ||
                  (control->ultrasonic->distance > usLimit)
              ? usLimit
              : control->ultrasonic->distance;
  PT0 = isnan(control->phototransistors->sensors[0].value)
            ? ptLimit
            : control->phototransistors->sensors[0].value;
  PT1 = isnan(control->phototransistors->sensors[1].value)
            ? ptLimit
            : control->phototransistors->sensors[1].value;
  PT2 = isnan(control->phototransistors->sensors[2].value)
            ? ptLimit
            : control->phototransistors->sensors[2].value;
  PT3 = isnan(control->phototransistors->sensors[3].value)
            ? ptLimit
            : control->phototransistors->sensors[3].value;
  gyro = control->gyro->currentAngle;

  if (updateStallDetection(gyro)) {
    control->moveDirection(0, 0, 0, 0);
    delay(15);
    state6StartMs = 0;
    currentState = 6;
    return;
  }

  bool report = true;

  // Filter IR Values
  filteredIR0 = Helpers::movingAverage(IR0, filteredIR0, 5, 1);
  filteredIR1 = Helpers::movingAverage(IR1, filteredIR1, 5, 1);
  filteredIR2 = Helpers::movingAverage(IR2, filteredIR2, 5, 1);
  filteredIR3 = Helpers::movingAverage(IR3, filteredIR3, 5, 1);
  filteredUltra = Helpers::movingAverage(ultra, filteredUltra, 3, 1);
  filteredPT0 = Helpers::movingAverage(PT0, filteredPT0, 5, 1);
  filteredPT1 = Helpers::movingAverage(PT1, filteredPT1, 5, 1);
  filteredPT2 = Helpers::movingAverage(PT2, filteredPT2, 5, 1);
  filteredPT3 = Helpers::movingAverage(PT3, filteredPT3, 5, 1);
  filteredGyro = Helpers::movingAverage(gyro, filteredGyro, 1.1, 1);

  float innerPTSum = filteredPT1 + filteredPT2;
  float innerPTDiff = filteredPT2 - filteredPT1;
  float outerPTSum = filteredPT0 + filteredPT3;
  float outerPTDiff = filteredPT3 - filteredPT0;
  float innerControl = 0.0f;
  float outerControl = 0.0f;
  float correction = 0.0f;

  if (innerPTSum > 1) {
    innerControl = 0.00013f * pow(innerPTSum, 1.03f) * innerPTDiff;
  }

  if (outerPTSum > 1) {
    outerControl = 0.0f * outerPTDiff / (outerPTSum * outerPTSum);
  }

  correction = innerControl + outerControl;

  correction = constrain(correction, -1024, 1024);

  // if (innerPTSum < 600) {
  //   correction *= 0.1;
  //   if (report) {
  //     Helpers::telePrint("IC_low", innerControl * 0.1f);
  //     // Helpers::telePrint("OC_low", outerControl * 0.1f);
  //   }
  // }

  if (innerPTSum > 200) {
    avoid = 0;
  }

  float correctionW = 0.0f;
  float correctionAbs = fabs(correction);
  if (correctionAbs > 0.0f) {
    float correctionMag = constrain(correctionAbs / 1024.0f, 0.0f, 1.0f);
    float wMag = 1.0f * correctionMag;
    float wSign = (correction > 0.0f) ? -1.0f : 1.0f;
    correctionW = wSign * wMag;
  }

  if (report) {
    // Helpers::telePrint("IC", innerControl);
    // Helpers::telePrint("OC", outerControl);
  }

  const bool leftFrontBlocked = filteredIR0 < -115.0f;
  const bool rightFrontBlocked = filteredIR3 < -115.0f;
  const bool frontIrTrap = leftFrontBlocked && rightFrontBlocked;
  if (i % 5 == 0) {
    FuzzyLogic::control(filteredUltra,  // Front
                        filteredIR1,    // Left
                        filteredIR2,    // Right
                        filteredIR0,    // Left Front
                        filteredIR3,    // Right Front
                        0,              // Gyro
                        0,              // Correction
                        this->x, this->y, this->w, avoid, rot_count, 0);
  }

  // Persistent wall-bypass direction lock:
  // if front is still blocked, keep moving in the chosen lateral direction
  // until the front clears. If that side blocks, flip once and persist.
  const bool nearFireApproachPreferred =
      /*(filteredUltra < 135.0f) && (innerPTSum > 70.0f) && !frontIrTrap;*/ false;
  const bool frontBlockedForBypass =
      avoid && !nearFireApproachPreferred &&
      ((filteredUltra < 100.0f) || frontIrTrap || (filteredIR0 < -150.0f) ||
       (filteredIR3 < -150.0f));
  const bool leftSideBlocked = filteredIR1 < -420.0f;
  const bool rightSideBlocked = filteredIR2 < -420.0f;

  if (frontBlockedForBypass) {
    if (wallBypassDir == 0.0f) {
      float requestedDir = Helpers::sgn(*y);
      if (requestedDir != 0.0f) {
        wallBypassDir = requestedDir;
      } else {
        float leftBlock = -filteredIR0;
        float rightBlock = -filteredIR3;
        wallBypassDir = (leftBlock >= rightBlock) ? 1.0f : -1.0f;
      }
    }

    if (wallBypassDir > 0.0f && rightSideBlocked && !leftSideBlocked) {
      wallBypassDir = -1.0f;
    } else if (wallBypassDir < 0.0f && leftSideBlocked && !rightSideBlocked) {
      wallBypassDir = 1.0f;
    }

    // Limit the turning command during bypass to avoid excessive rotation that could cause the robot to lose sight of the fire or get stuck. The bypass direction is maintained to ensure consistent lateral movement away from the obstacle until it is cleared, even if the correction suggests a stronger turn
    float lowerLimit = 0.003f;
    float upperLimit = lowerLimit;

    if (correctionW < 0.0f) {
      correctionW = constrain(correctionW, -upperLimit, -lowerLimit);
    } else if (correctionW >= 0.0f) {
      correctionW = constrain(correctionW, lowerLimit, upperLimit);
    }

    const float bypassW = correctionW;
    const float bypassY = 0.6f * wallBypassDir;

    if (wallBypassDir == -1.0f) {
      if (filteredUltra < 85.0f || filteredIR0 < -180.0f || filteredIR3 < -180.0f) {
        *x = -0.1f;
        *y = bypassY;
      } else {
        *x = 0.1f;
        *y = bypassY;
      }
      *w = 1.0f* bypassW;
    }
    if (wallBypassDir == 1.0f) {
      if (filteredUltra < 85.0f || filteredIR0 < -180.0f || filteredIR3 < -180.0f) {
        *x = -0.1f;
        *y = bypassY;
      } else {
        *x = 0.1f;
        *y = bypassY;
      }
      *w = bypassW * 1.0f;
    }
  } else {
    wallBypassDir = 0.0f;

    // Adjust *w based on how strong innerPTSum is (higher sum means lower correction to avoid overshooting when close to fire), linearly inverse scaling formula. correctionW is not used.
    float minCorrectionW = 0.0011f;
    float maxCorrectionW = 0.0015f;
    float scaledCorrectionW = maxCorrectionW;
    if (innerPTSum > 1) {
      float scalingFactor = 1.0f - (innerPTSum / 1024.0f);
      scalingFactor = constrain(scalingFactor, 0.1f, 1.0f);
      scaledCorrectionW = minCorrectionW + (maxCorrectionW - minCorrectionW) * scalingFactor;
    }
    *w = scaledCorrectionW * Helpers::sgn(correction) * -1.0f;
    
  }

  const bool severeObstacle = false;
  //     frontIrTrap ||
  //     (filteredUltra < 120.0f && (leftFrontBlocked || rightFrontBlocked)) ||
  //     (leftSideBlocked && rightSideBlocked);

  // if (nearFireApproachPreferred && !severeObstacle) {
  //   // Close to a confirmed target: bias toward straight approach while still
  //   // allowing small lateral/heading corrections.
  //   const float minForward = (filteredUltra < 105.0f) ? 0.18f : 0.28f;
  //   if (*x < minForward) {
  //     *x = minForward;
  //   }
  //   *y = constrain(*y, -0.16f, 0.16f);
  //   *w = constrain(correctionW, -0.18f, 0.18f);
  // } else if (!frontBlockedForBypass) {
  //   // Keep bypass-phase w command untouched so bypassW gains are effective.
  //   *w = correctionW;
  // }

  // Fire-capture gate: once the fire is confidently close, stop advancing and
  // hand off to the extinguish state instead of driving into the flame.
  // This keeps the existing fuzzy/FSM behavior but adds a clean terminal step.
  // const bool fireCaptureReady =
  //     (innerPTSum > 250.0f && filteredUltra < 110.0f) ||
  //     (filteredPT1 + filteredPT2 > 370.0f);
  // if (fireCaptureReady) {
  //   *x = 0.0f;
  //   *y = 0.0f;
  //   *w = 0.0f;
  //   control->moveDirection(0, 0, 0, 0);
  //   currentState = 4;
  //   return;
  // }

  // Gradually increase speed as robot approaches fire (avoid jolt)
  int speed = 367;
  if (!avoid) {
    speed = 305;
  }

  control->moveDirection(*x, *y, *w, speed);
  i++;

  // Use filtered values only to avoid noisy premature transitions
  unsigned long nowMs = millis();
  const unsigned long fireLostGraceMs = 1000;
  bool closeToFire = filteredUltra < 130.0f;
  bool seesFireNow = (innerPTSum > 10);
  if (seesFireNow) {
    state3LastFireSeenMs = nowMs;
  }
  bool recentlySawFire = state3LastFireSeenMs != 0 &&
                         nowMs - state3LastFireSeenMs <= fireLostGraceMs;
  bool seesFire = seesFireNow || recentlySawFire;
  bool ExtinguishFire = innerPTSum > 250;

  // Exit state when close to the located light source
  if (ExtinguishFire && closeToFire || (filteredPT1 + filteredPT2 > 350.0)) {
    // Stop

    delay(15);
    control->moveDirection(0, 0, 0, 0);

    currentState = 4;
  }

  // If lose light
  if (!seesFire) {
    control->moveDirection(0, 0, 0, 0);
    state2AimLeftToRight = !state2AimLeftToRight;
    currentState = 1;
  }
}

void FSM::state4_extinguish() {
  // Extinguish the fire
  bool firstCycle = (enterTime == 0);
  static int state4LockedServoAngle = 0;
  static unsigned long state4ServoStableMs = 0;
  static bool state4PreFanAdvanceDone = false;
  static unsigned long state4FanOnMs = 0;
  if (firstCycle) {
    enterTime = i;
    state4AlignStartMs = 0;
    state4LockedServoAngle = 0;
    state4ServoStableMs = 0;
    state4PreFanAdvanceDone = false;
    state4FanOnMs = 0;
    control->setMotionLock(true);
    control->fanServo->set_angle(0, 2);
  }

  // Hard hold while extinguishing, except for the pre-fan advance.
  control->moveDirection(0, 0, 0, 0);

  control->read(i);

  float ptLimit = 1024;
  float PT1 = isnan(control->phototransistors->sensors[1].value)
                  ? ptLimit
                  : control->phototransistors->sensors[1].value;
  float PT2 = isnan(control->phototransistors->sensors[2].value)
                  ? ptLimit
                  : control->phototransistors->sensors[2].value;

  bool report = true;

  filteredPT1 = Helpers::movingAverage(PT1, filteredPT1, 5, 1);
  filteredPT2 = Helpers::movingAverage(PT2, filteredPT2, 5, 1);

  unsigned long nowMs = millis();
  if (state4AlignStartMs == 0) {
    state4AlignStartMs = nowMs;
  }
  bool alignPhase = (nowMs - state4AlignStartMs) < 100UL;
  if (alignPhase) {
    // Refresh PT readings during alignment to avoid stale values.
    control->phototransistors->read(-1);
    float rawPT1 = isnan(control->phototransistors->sensors[1].value)
                       ? ptLimit
                       : control->phototransistors->sensors[1].value;
    float rawPT2 = isnan(control->phototransistors->sensors[2].value)
                       ? ptLimit
                       : control->phototransistors->sensors[2].value;

    filteredPT1 = Helpers::movingAverage(rawPT1, filteredPT1, 5, 1);
    filteredPT2 = Helpers::movingAverage(rawPT2, filteredPT2, 5, 1);

    float ptSum = filteredPT1 + filteredPT2;
    float diff = filteredPT2 - filteredPT1;
    float innerControl = 0.0f;
    float outerControl = 0.0f;
    float correction = 0.0f;
    if (ptSum > 1) {
      innerControl = 16500.0f * diff / (ptSum * ptSum);
    }

    correction = innerControl + outerControl;

    int servoAngle =
        (int)constrain((correction / 1024.0f), -75.0f, 75.0f);
    state4LockedServoAngle = servoAngle;
    state4ServoStableMs = nowMs;
    control->fanServo->set_angle(servoAngle, 2);
    if (report) {
      Helpers::telePrint("IC", innerControl);
      Helpers::telePrint("OC", outerControl);
    }
    enterTime = i;
    control->fan->set_status(0);
  } else {
    control->fanServo->set_angle(state4LockedServoAngle, 2);

    // Wait for the servo to settle before starting the fan, so the air stream
    // begins only after the nozzle is already aimed.
    const bool servoSettled = (nowMs - state4ServoStableMs) >= 150UL;
    if (servoSettled && !state4PreFanAdvanceDone) {
      const unsigned long nudgeMs = 5UL;
      const float stopDistance = 20.0f;
      const float usLimit = 1300.0f;
      const float irLimit = 1024.0f;
      const float irSideTooClose = 450.0f;
      const unsigned long reaimTurnMs = 140UL;
      control->setMotionLock(false);
      control->irSensors->read(-1);
      float leftIr = isnan(control->irSensors->sensors[1].distance)
                         ? irLimit
                         : control->irSensors->sensors[1].distance;
      float rightIr = isnan(control->irSensors->sensors[2].distance)
                          ? irLimit
                          : control->irSensors->sensors[2].distance;
      bool leftTooClose = leftIr > irSideTooClose;
      bool rightTooClose = rightIr > irSideTooClose;

      if (leftTooClose || rightTooClose) {
        float lateralDir = 0.0f;
        float rotateDir = 0.0f;
        if (leftTooClose && !rightTooClose) {
          lateralDir = 1.0f;
          rotateDir = 1.0f;
        } else if (rightTooClose && !leftTooClose) {
          lateralDir = -1.0f;
          rotateDir = -1.0f;
        } else {
          bool leftIsCloser = leftIr >= rightIr;
          lateralDir = leftIsCloser ? -1.0f : 1.0f;
          rotateDir = leftIsCloser ? -1.0f : 1.0f;
        }

        control->moveDirection(0, lateralDir, 0, 200);
        delay(300);
        control->moveDirection(0, 0, rotateDir, 200);
        delay(reaimTurnMs);
        control->moveDirection(0, 0, 0, 0);
      }

      unsigned long nudgeStartMs = millis();
      control->moveDirection(1, 0, 0, 0);
      while (millis() - nudgeStartMs < nudgeMs) {
        control->ultrasonic->read();
        float nudgeUltra =
            isnan(control->ultrasonic->distance)
                ? usLimit
                : control->ultrasonic->distance;
        if (nudgeUltra <= stopDistance) {
          break;
        }
        delay(10);
      }
      control->moveDirection(0, 0, 0, 0);
      control->setMotionLock(true);
      state4PreFanAdvanceDone = true;
    }
    control->fan->set_status(servoSettled ? 1 : 0);
    if (servoSettled && state4FanOnMs == 0) {
      state4FanOnMs = nowMs;
    }

    if (servoSettled) {
      const unsigned long fireOutTimeoutMs = 10000UL;
      const float fireOutThreshold = 10.0f;
      while ((millis() - state4AlignStartMs) < fireOutTimeoutMs) {
        unsigned long loopNowMs = millis();
        control->phototransistors->read(-1);
        float rawPT1 = isnan(control->phototransistors->sensors[1].value)
                           ? ptLimit
                           : control->phototransistors->sensors[1].value;
        float rawPT2 = isnan(control->phototransistors->sensors[2].value)
                           ? ptLimit
                           : control->phototransistors->sensors[2].value;

        filteredPT1 = Helpers::movingAverage(rawPT1, filteredPT1, 5, 1);
        filteredPT2 = Helpers::movingAverage(rawPT2, filteredPT2, 5, 1);

        float ptSum = filteredPT1 + filteredPT2;
        if ((loopNowMs - state4FanOnMs) < 300UL) {
          delay(10);
          i++;
          continue;
        }
        if (ptSum < fireOutThreshold) {
          control->fan->set_status(0);
          control->setMotionLock(false);

          num_fires--;

          if (num_fires == 0) {
            status = STOPPED;
          }

          currentState = 5;
          enterTime = 0;
          state4AlignStartMs = 0;
          return;
        }

        delay(10);
        i++;
      }
    }
  }

  int dwellTime = i - enterTime;
  bool fanHasRunLongEnough = (nowMs - state4AlignStartMs) >= 10000UL;
  bool fireOutNow = (filteredPT1 + filteredPT2 < 10.0f);

  // As soon as the light source is out, stop the fan and release the motion
  // lock immediately.
  if (fanHasRunLongEnough && fireOutNow) {
    control->fan->set_status(0);
    control->setMotionLock(false);

    num_fires--;

    if (num_fires == 0) {
      status = STOPPED;
    }

    currentState = 5;
    enterTime = 0;
    state4AlignStartMs = 0;
    return;
  }

  // Require minimum dwell time before checking if fire is out
  if (fanHasRunLongEnough && (dwellTime > 1000)) {
    // Turn off fan
    control->fan->set_status(0);
    control->setMotionLock(false);

    num_fires--;

    if (num_fires == 0) {
      // Stop
      status = STOPPED;
    }

    // Restart
    currentState = 5;
    enterTime = 0;
    state4AlignStartMs = 0;
  }

  i++;
}

void FSM::state5_reverse() {
  // Debug: capture sensor state right before reversing
  Helpers::telePrint("State", "5_reverse");
  Helpers::telePrint("StateId", currentState);
  Helpers::telePrint("i", i);

  control->gyro->update();
  control->irSensors->read(-1);
  control->phototransistors->read(-1);
  control->ultrasonic->read();

  float irLimit = 1024;
  float usLimit = 1300;
  float ptLimit = 1024;

  float IR0 = isnan(control->irSensors->sensors[0].distance) ||
                      (control->irSensors->sensors[0].distance > irLimit)
                  ? -irLimit
                  : -control->irSensors->sensors[0].distance;
  float IR1 = isnan(control->irSensors->sensors[1].distance) ||
                      (control->irSensors->sensors[1].distance > irLimit)
                  ? -irLimit
                  : -control->irSensors->sensors[1].distance;
  float IR2 = isnan(control->irSensors->sensors[2].distance) ||
                      (control->irSensors->sensors[2].distance > irLimit)
                  ? -irLimit
                  : -control->irSensors->sensors[2].distance;
  float IR3 = isnan(control->irSensors->sensors[3].distance) ||
                      (control->irSensors->sensors[3].distance > irLimit)
                  ? -irLimit
                  : -control->irSensors->sensors[3].distance;
  float ultra = isnan(control->ultrasonic->distance) ||
                        (control->ultrasonic->distance > usLimit)
                    ? usLimit
                    : control->ultrasonic->distance;
  float PT1 = isnan(control->phototransistors->sensors[1].value)
                  ? ptLimit
                  : control->phototransistors->sensors[1].value;
  float PT2 = isnan(control->phototransistors->sensors[2].value)
                  ? ptLimit
                  : control->phototransistors->sensors[2].value;

  Helpers::telePrint("S5IR0", IR0);
  Helpers::telePrint("S5IR1", IR1);
  Helpers::telePrint("S5IR2", IR2);
  Helpers::telePrint("S5IR3", IR3);
  Helpers::telePrint("S5Ultra", ultra);
  Helpers::telePrint("S5PTsum", PT1 + PT2);

  Helpers::telePrint("S5Action", "reverse");
  // Reverse
  control->moveDirection(-1, 0, 0, 250);
  delay(50);
  control->moveDirection(0, 0, 0, 0);
  Helpers::telePrint("S5Transition", "restart");
  enterTime = 0;
  currentState = 1;
}

void FSM::state6_stalled() {
  unsigned long nowMs = millis();
  if (state6StartMs == 0) {
    state6StartMs = nowMs;
  }

  float strafeDir = wallBypassDir;
  if (strafeDir == 0.0f) {
    strafeDir = Helpers::sgn(*y);
  }
  if (strafeDir == 0.0f) {
    strafeDir = 1.0f;
  }

  control->moveDirection(-2.0, strafeDir * 0.8, 0, 300);

  if ((nowMs - state6StartMs) >= 400UL) {
    control->moveDirection(0, 0, 0, 0);
    resetStallDetection();
    state6StartMs = 0;
    avoid = 1;
    currentState = 3;
  }
}
