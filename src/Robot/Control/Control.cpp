#include "Control.h"

#include "../../Misc/Helpers/Helpers.h"

/* CONSTRUCTOR */
Control::Control(Motors* motors, Fan* fan, FanServo* fanServo, Gyro* gyro,
                 IRSensors* irSensors, Ultrasonic* ultrasonic,
                 Phototransistors* phototransistors) {
  this->motors = motors;
  this->fan = fan;
  this->fanServo = fanServo;

  this->gyro = gyro;
  this->irSensors = irSensors;
  this->ultrasonic = ultrasonic;
  this->phototransistors = phototransistors;
}

void Control::enable() {
  motors->enable();
  fanServo->enable();
  fan->enable();

  gyro->enable();
  irSensors->enable();
  ultrasonic->enable();
  phototransistors->enable();
}

void Control::disable() {
  motors->disable();
  fanServo->disable();
  fan->disable();

  gyro->disable();
  irSensors->disable();
  ultrasonic->disable();
  phototransistors->disable();
}

void Control::setMotionLock(bool locked) {
  this->motionLocked = locked;
  if (locked) {
    for (int i = 0; i < 4; i++) {
      this->motors->set_speed(i + 1, 0);
    }
  }
}

bool Control::isMotionLocked() const { return this->motionLocked; }

void Control::read(int i) {
  this->gyro->update();
  // Always sample IR sensors for timely obstacle detection
  this->irSensors->read(-1);
  // Phototransistors sampled at a lower rate to reduce load
  if (i % 5 == 0) {
    this->phototransistors->read(-1);
  }

  if (i % 5 == 0) {
    this->ultrasonic->read();
  }
}

void Control::moveDirection(double x, double y, double w, double speed)
// move in direction by proportion of x,y,w and a scalar speed
//  implements inverse kinematics of system
{
  if (this->motionLocked) {
    for (int i = 0; i < 4; i++) {
      this->motors->set_speed(i + 1, 0);
    }
    return;
  }

  // +x is forwards, +y is right, +w is clockwise
  // clamp inputs between -1 & 1
  // this constrain is not needed as ratio is preserved and maximum is set to
  // speed but useful for "normalising"
  x = constrain(x, -1, 1);
  y = constrain(y, -1, 1);
  w = constrain(w, -1, 1);

  // Command deadband: avoid powering motors with tiny ineffective effort
  // that causes buzzing/stall near goal states.
  double commandMag = max(abs(x), max(abs(y), abs(w)));
  if (commandMag < 0.05 || speed <= 0) {
    for (int i = 0; i < 4; i++) {
      this->motors->set_speed(i + 1, 0);
    }
    return;
  }

  speed = commandMag * speed;

  // Ensure commanded motion clears static friction once movement is requested.
  const double kMinEffectiveSpeed = 85.0;
  if (speed > 0 && speed < kMinEffectiveSpeed) {
    speed = kMinEffectiveSpeed;
  }

  // constrain speed to max 500
  speed = constrain(speed, -500, 500);

  // CORRECTIONS
  double w5 = 0.00063;
  double w2 = 0.00022;

  if (x > 0) {
    w += ((w5 - w2) / (300)) * (x - 200) + w2;
  } else {
    w += ((0.00008 + 0.0003) / (300)) * (x + 200) + 0.00008;
  }

  int L = 105;     // Vert Wheel Distance
  int l = 90;      // Horiz Wheel Distance
  double Rw = 30;  // Wheel Radius

  // Inverse kinematics
  float theta[4] = {0, 0, 0, 0};
  theta[0] = (1.0 / Rw) * (x + y - (L + l) * w);
  theta[1] = (1.0 / Rw) * (x - y + (L + l) * w);
  theta[2] = (1.0 / Rw) * (x - y - (L + l) * w);
  theta[3] = (1.0 / Rw) * (x + y + (L + l) * w);

  // Cap speed val so maximum set servo speed cannot exceed 500
  //   ensures that direction is as desired
  double max = 0;
  for (int i = 0; i < 4; i++)
  // extract maximum theta value
  {
    double absTheta = abs(theta[i]);
    max = absTheta > max ? absTheta : max;
  }

  if (max < 1e-6) {
    for (int i = 0; i < 4; i++) {
      this->motors->set_speed(i + 1, 0);
    }
    return;
  }

  // extract the factor by which to multiply theta to ensure the desired speed
  // is attributed to the highest theta (and maintain ratios)
  double ratio = ((double)speed / max);

  // Set motor speed
  for (int i = 0; i < 4; i++) {
    this->motors->set_speed(i + 1, ratio * theta[i]);
  }
}

void Control::moveDirectionCapX(double x, double y, double w)
// move in x y w direction but preserve the y and w terms such that they are
// maintained if x produces a capped speed +x is forwards, +y is right, +w is
// clockwise
{
  if (this->motionLocked) {
    for (int i = 0; i < 4; i++) {
      this->motors->set_speed(i + 1, 0);
    }
    return;
  }

  double L = 105;  // Vert Wheel Distance
  double l = 90;   // Horiz Wheel Distance
  double Rw = 30;  // Wheel Radius

  // if moving in x direction with long IR sensors, reduce the y effort
  // y = longIRControl == 2 || longIRControl == 3 ? y/3 : y;

  // find the inverse kinematics of only the y and w terms
  double theta[4] = {0, 0, 0, 0};
  theta[0] = (1.0 / Rw) * (0 + y - (L + l) * w);
  theta[1] = (1.0 / Rw) * (0 - y + (L + l) * w);
  theta[2] = (1.0 / Rw) * (0 - y - (L + l) * w);
  theta[3] = (1.0 / Rw) * (0 + y + (L + l) * w);

  // scale the yw input to ensure they are not at the +-500 motor limits (find
  // max/min and simply scale) (speed limiter) (NOTIFY USER IF CAPPED INPUT)
  double max = 0;
  for (int i = 0; i < 4; i++)
  // extract maximum theta value
  {
    double absTheta = abs(theta[i]);
    max = absTheta > max ? absTheta : max;
  }

  double ratio;
  if (max > 500) {
    // extract the factor by which to multiply theta to ensure the maximum speed
    // is within +-500 (and maintain ratios)
    ratio = (500 / max);
  } else {
    ratio = 1;
  }

  // scale the yw theta values
  for (int i = 0; i < 4; i++) {
    theta[i] = ratio * theta[i];
  }

  // determine the max and min input values, find the difference at +- 500
  // (speed limiter)
  max = 0;
  double min = 0;
  for (int i = 0; i < 4; i++) {
    max = theta[i] > max ? theta[i] : max;
    min = theta[i] < min ? theta[i] : min;
  }
  double maxAllowableTheta = 500 - max;
  double minAllowableTheta = -500 - min;

  // apply forward kinematics to only the first row to determine the upper and
  // lower x effort
  double maxAllowableX = (Rw / 4.0) * (4.0 * maxAllowableTheta);
  double minAllowableX = (Rw / 4.0) * (4.0 * minAllowableTheta);

  // apply the upper and lower constraint to the x effort
  x = constrain(x, minAllowableX, maxAllowableX);

  // CORRECTIONS (untested for driving straight)
  if (x > 0) {
    w += ((0.0007 - 0.00022) / (300)) * (x - 200) + 0.00022;
  } else {
    w += ((0.00008 + 0.0003) / (300)) * (x + 200) + 0.00008;
  }

  // now apply inverse kinematics to the total effort with constrained x
  theta[0] = (1.0 / Rw) * (x + y - (L + l) * w);
  theta[1] = (1.0 / Rw) * (x - y + (L + l) * w);
  theta[2] = (1.0 / Rw) * (x - y - (L + l) * w);
  theta[3] = (1.0 / Rw) * (x + y + (L + l) * w);

  // note: the control input is within +-500 due to the previous calculations so
  // no speed value is necessary. The x value will go to the maximum allowed
  // when there is little y/w term Set motor speed
  //
  for (int i = 0; i < 4; i++) {
    this->motors->set_speed(i + 1, theta[i]);
  }
}

void Control::controlAngle(double angle) {
  double Kp = 12.5;  // Proportional gain
  double Ki = 0.05;  // Integral gain
  Ki = 0;
  double threshold = 10.0;  // Threshold for angle error
  double error;
  double area = 0;  // Integral area

  int withinThreshold = 0;
  double initialTime;
  double timeElapsed = 0;

  // FOR PRINTING
  int count = 0;

  // Continue turning until the angle error is within the threshold
  do {
    count++;
    // update the gyro angle
    this->gyro->update();

    error = angle - this->gyro->currentAngle;  // Update the error
    area += error;                             // Update the integral area
    double u_p = Kp * error;
    double u_i = Ki * area;
    double u = u_p + u_i;  // Calculate the control input

    u = constrain(u, -550, 550);

    if (abs(u) < 78) {
      u = Helpers::sgn(u) * 78;
    }

    if (count == 1 || count % 10 == 0) {
      Helpers::telePrint("         TurnTarget", angle);
      Helpers::telePrint("         TurnAngle", this->gyro->currentAngle);
      Helpers::telePrint("         TurnError", error);
      Helpers::telePrint("         TurnU", u);
    }

    // Set motor speed to turn the robot
    this->moveDirection(0, 0, Helpers::sgn(u), abs(u));
    delay(1);

    // exit control loop if within threshold for 2 seconds
    if (abs(error) < threshold) {
      withinThreshold++;
    }
  } while (withinThreshold < 20);

  // stop motors when done
  this->moveDirection(0, 0, 1, 0);
}

// controls robot to aim at the brightest light source
void Control::aim(int sweepAngle, bool leftToRight, bool allowWeakRetry) {
  int offsetAngle = 40;   // side PT angle offset
  int edgeThreshold = 5;  // angle from edge to ignore maximum PT value found
  const int kWeakAimThreshold = 14;

  const int kPTFarLeft = 0;
  const int kPTMidLeft = 1;
  const int kPTMidRight = 2;
  const int kPTFarRight = 3;

  int startAngle = leftToRight ? -sweepAngle : sweepAngle;

  this->fanServo->set_angle(startAngle, 2);

  // filtering stuff
  int adcVal1;
  int adcVal2;
  int adcVal3;
  int adcVal4;
  int adcVal1_p = 0;
  int adcVal2_p = 0;
  int adcVal3_p = 0;
  int adcVal4_p = 0;

  for (int i = 0; i < 100; i++) {
    this->phototransistors->read(-1);
    // PT1=FarLeft, PT2=MidLeft, PT3=MidRight, PT4=FarRight
    adcVal1 = this->phototransistors->sensors[kPTFarLeft].value;
    adcVal2 = this->phototransistors->sensors[kPTMidLeft].value;
    adcVal3 = this->phototransistors->sensors[kPTMidRight].value;
    adcVal4 = this->phototransistors->sensors[kPTFarRight].value;

    int raw1 = adcVal1;
    int raw2 = adcVal2;
    int raw3 = adcVal3;
    int raw4 = adcVal4;

    // filter the PT values
    adcVal1 = Helpers::movingAverage(adcVal1, adcVal1_p, 3, 1);
    adcVal2 = Helpers::movingAverage(adcVal2, adcVal2_p, 3, 1);
    adcVal3 = Helpers::movingAverage(adcVal3, adcVal3_p, 3, 1);
    adcVal4 = Helpers::movingAverage(adcVal4, adcVal4_p, 3, 1);

    adcVal1_p = adcVal1;
    adcVal2_p = adcVal2;
    adcVal3_p = adcVal3;
    adcVal4_p = adcVal4;

    if (i % 10 == 0) {
      Helpers::telePrint("AimPTsum", raw2 + raw3);
      Helpers::telePrint("AimFPTsum", adcVal2 + adcVal3);
    }

    delay(1);
  }

  auto scanForMax = [&](bool sweepLeftToRight, int& max_combined,
                        int& max_angle_combined) -> bool {
    int startAngleLocal = sweepLeftToRight ? -sweepAngle : sweepAngle;
    int endAngleLocal = sweepLeftToRight ? sweepAngle : -sweepAngle;
    int sweepStepLocal = sweepLeftToRight ? 1 : -1;

    this->fanServo->set_angle(startAngleLocal, 2);

    int PT_values_combined[(sweepAngle + offsetAngle) * 2 +
                           1];  // -105 to 105 (index 0 to 210)

    // sweep and record the PT values for each angle
    for (int i = startAngleLocal;
         sweepLeftToRight ? i <= endAngleLocal : i >= endAngleLocal;
         i += sweepStepLocal) {
      this->fanServo->set_angle(i);

      this->phototransistors->read(-1);
      // PT1=FarLeft, PT2=MidLeft, PT3=MidRight, PT4=FarRight
      adcVal1 = this->phototransistors->sensors[kPTFarLeft].value;
      adcVal2 = this->phototransistors->sensors[kPTMidLeft].value;
      adcVal3 = this->phototransistors->sensors[kPTMidRight].value;
      adcVal4 = this->phototransistors->sensors[kPTFarRight].value;

      int raw1 = adcVal1;
      int raw2 = adcVal2;
      int raw3 = adcVal3;
      int raw4 = adcVal4;

      // filter the PT values
      adcVal1 = Helpers::movingAverage(adcVal1, adcVal1_p, 3, 1);
      adcVal2 = Helpers::movingAverage(adcVal2, adcVal2_p, 3, 1);
      adcVal3 = Helpers::movingAverage(adcVal3, adcVal3_p, 3, 1);
      adcVal4 = Helpers::movingAverage(adcVal4, adcVal4_p, 3, 1);

      adcVal1_p = adcVal1;  // fl
      adcVal2_p = adcVal2;  // cl
      adcVal3_p = adcVal3;  // cr
      adcVal4_p = adcVal4;  // fr

      if (i % 10 == 0) {
        Helpers::telePrint("AimPTsum", raw2 + raw3);
        Helpers::telePrint("AimFPTsum", adcVal2 + adcVal3);
      }

      // combined mode
      // store centre PT values from -75 to 75 (index of 30 to 180)
      PT_values_combined[i + sweepAngle + offsetAngle] =
          (adcVal2 + adcVal3) / 2;
      if (i < -sweepAngle + offsetAngle) {
        // store far left PT values from -105 to -76 (index of 0 to 29)
        PT_values_combined[i + sweepAngle] = adcVal1;
      }
      if (i > sweepAngle - offsetAngle) {
        // store far right PT values from 76 to 105 (index of 181 to 210)
        PT_values_combined[(sweepAngle + offsetAngle) * 2 + 1 - offsetAngle +
                           (i - (sweepAngle - offsetAngle + 1))] = adcVal4;
      }
    }

    // find the maximum PT value and its angle
    max_combined = 0;
    max_angle_combined = 0;
    for (int i = 0; i < (sweepAngle + offsetAngle) * 2 + 1; i++) {
      if (PT_values_combined[i] > max_combined) {
        max_combined = PT_values_combined[i];
        max_angle_combined = i - sweepAngle - offsetAngle;
      }
    }

    Helpers::telePrint("AimMax", max_combined);
    Helpers::telePrint("AimMaxAng", max_angle_combined);

    int localFireFound = 0;
    // determine if a fire is found within -110 and 110 degrees
    if (max_combined >= 3 &
        max_angle_combined >= -sweepAngle + edgeThreshold - offsetAngle &
        max_angle_combined <= sweepAngle - edgeThreshold + offsetAngle) {
      // fire found between -110 and 110
      localFireFound = 1;
    }

    Helpers::telePrint("AimFound", localFireFound);

    return localFireFound == 1;
  };

  int max_combined = 0;
  int max_angle_combined = 0;
  bool fireFound = scanForMax(leftToRight, max_combined, max_angle_combined);
  bool activeLeftToRight = leftToRight;

  if (allowWeakRetry && fireFound && max_combined < kWeakAimThreshold) {
    int initial_max_combined = max_combined;
    int initial_max_angle = max_angle_combined;
    int retriesPerformed = 0;
    bool strongerSourceFound = false;

    // Keep rotating through the remaining terrain sectors until a stronger
    // fire appears, or we've checked both additional sweeps.
    for (int retryIndex = 0; retryIndex < 2; retryIndex++) {
      this->gyro->reset();
      this->controlAngle(120);
      this->gyro->reset();
      retriesPerformed++;

      int retry_max_combined = 0;
      int retry_max_angle = 0;
      bool retryLeftToRight = (retryIndex % 2 == 0) ? !leftToRight : leftToRight;
      bool retryFireFound =
          scanForMax(retryLeftToRight, retry_max_combined, retry_max_angle);

      if (retryFireFound && retry_max_combined > initial_max_combined) {
        max_combined = retry_max_combined;
        max_angle_combined = retry_max_angle;
        fireFound = true;
        activeLeftToRight = retryLeftToRight;
        strongerSourceFound = true;
        break;
      }
    }

    if (!strongerSourceFound) {
      // No stronger source found after scanning all sectors, return to the
      // initial aim.
      this->gyro->reset();
      this->controlAngle(-120 * retriesPerformed);
      this->gyro->reset();

      max_combined = initial_max_combined;
      max_angle_combined = initial_max_angle;
      fireFound = true;
      activeLeftToRight = leftToRight;
    }
  }

  if (!fireFound) {
    // otherwise, turn the robot 120 degrees and try again
    this->gyro->reset();
    this->controlAngle(120);
    this->gyro->reset();
    this->aim(sweepAngle, !leftToRight, allowWeakRetry);
  }

  if (fireFound) {
    double turnAngle = -max_angle_combined * 0.5;
    double centerTurnAngle = -(max_angle_combined - 5) * 0.5;

    // Centre the fan before rotating the robot so it does not stay at the sweep
    // angle.
    this->fanServo->set_angle(0, 2);

    // centre section defined as -70 to 70
    if (max_angle_combined > -sweepAngle + edgeThreshold &
        max_angle_combined < sweepAngle - edgeThreshold) {
      this->gyro->reset();
      this->controlAngle(centerTurnAngle);
      this->gyro->reset();
    } else {
      // if not in the centre section, turn the robot to the angle of the
      // maximum PT value and perform sweep again
      this->gyro->reset();
      this->controlAngle(turnAngle);
      this->gyro->reset();
      this->aim(sweepAngle, activeLeftToRight, allowWeakRetry);
    }

    // reset servo to center
    this->fanServo->set_angle(0, 2);
  }
}
