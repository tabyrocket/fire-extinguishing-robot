#include "Fuzzy.h"

#include "../../../Misc/Helpers/Helpers.h"
#include "../../../Misc/Helpers/LED.h"

const FuzzySet FuzzyLogic::Ultrasonic_FS = {
    4,        {0, 25, 60}, {45, 70, 100}, {70, 110, 150}, {100, 450, 1300},
    {0, 0, 0}};

const FuzzySet FuzzyLogic::LeftFront_FS = {4,
                                           {-1024, -510, -290},
                                           {-400, -265, -220},
                                           {-220, 0, 0},
                                           {0, 0, 0}};

const FuzzySet FuzzyLogic::RightFront_FS = {4,
                                           {-1024, -510, -290},
                                           {-400, -265, -220},
                                           {-220, 0, 0},
                                           {0, 0, 0}};


const FuzzySet FuzzyLogic::Left_FS = {4,
                                      {-1024, -730, -600},
                                      {-680, -600, -525},
                                      {-580, -500, -375},
                                      {-475, -375, 0},
                                      {0, 0, 0}};

const FuzzySet FuzzyLogic::Right_FS = {4,
                                       {-1024, -730, -600},
                                       {-680, -600, -525},
                                       {-580, -500, -375},
                                       {-475, -375, 0},
                                       {0, 0, 0}};

const FuzzySet FuzzyLogic::Gyro_FS = {
    3, {-180, -90, -0.2}, {-3, 0, 3}, {0.2, 90, 180}, {0, 0, 0}, {0, 0, 0},
};

const FuzzySet FuzzyLogic::Correction_FS = {
    3,         {-1024, -1023, -50}, {-80, 0, 80}, {50, 1023, 1024},
    {0, 0, 0}, {0, 0, 0},
};

const FuzzySet FuzzyLogic::X_FS = {5,
                                   {-1, -0.9, -0.7},
                                   {-0.8, -0.5, -0.3},
                                   {-0.4, 0, 0.4},
                                   {0.3, 0.5, 0.8},
                                   {0.7, 0.9, 1}};

const FuzzySet FuzzyLogic::Y_FS = {5,
                                   {-1, -0.9, -0.6},
                                   {-0.8, -0.5, -0.3},
                                   {-0.4, 0, 0.4},
                                   {0.3, 0.5, 0.8},
                                   {0.6, 0.9, 1}};

const FuzzySet FuzzyLogic::W_FS = {5,
                                   {-1, -0.8, -0.05},
                                   {-0.6, -0.5, 0.6},
                                   {-0.4, 0, 0.4},
                                   {-0.6, 0.5, 0.6},
                                   {0.05, 0.8, 1}};

// Fuzzification Membership Functions
float FuzzyLogic::tri_fmf(float x, float p[3]) {
  if (x <= p[0] || x >= p[2]) return 0;
  if (x > p[0] && x <= p[1]) return (x - p[0]) / (p[1] - p[0]);
  return (p[2] - x) / (p[2] - p[1]);
}

float FuzzyLogic::lins_fmf(float x, float p[3]) {
  if (x <= p[0]) return 0;
  if (x >= p[1]) return 1;
  return (x - p[0]) / (p[1] - p[0]);
}

float FuzzyLogic::linz_fmf(float x, float p[3]) {
  if (x <= p[1]) return 1;
  if (x >= p[2]) return 0;
  return (p[2] - x) / (p[2] - p[1]);
}

// Defuzzification Membership Functions
FuzzyCentroid FuzzyLogic::tri_dmf(float h, float p[3]) {
  // Points
  float p0 = p[0];                         // Bottom Left
  float p0_5 = p[1] * h - p[0] * (h - 1);  // Top Left
  float p1_5 = p[1] * h - p[2] * (h - 1);  // Top Right
  float p2 = p[2];                         // Bottom Right

  // Centroids
  float u1 = (2.0 / 3) * (p0_5 - p0) + p0;
  float u2 = (1.0 / 2) * (p1_5 - p0_5) + p0_5;
  float u3 = (1.0 / 3) * (p2 - p1_5) + p1_5;

  // Areas
  float a1 = (1.0 / 2) * (p0_5 - p0) * h;
  float a2 = (p1_5 - p0_5) * h;
  float a3 = (1.0 / 2) * (p2 - p1_5) * h;

  // Overall Centroid
  FuzzyCentroid fc;
  fc.area = a1 + a2 + a3;
  if (fc.area != 0) {
    fc.centroid = (a1 * u1 + a2 * u2 + a3 * u3) / fc.area;
  } else {
    fc.centroid = 0;
  }

  return fc;
}

FuzzyCentroid FuzzyLogic::linz_dmf(float h, float p[3]) {
  float minVal = p[0];

  // Points
  float p0_5 = -(p[2] - p[1]) * (h - 1) + p[1];  // Top Left
  float p1 = p[2];                               // Bottom Right

  // Centroids
  float u1 = (1.0 / 2) * (p0_5 + minVal);

  float u2 = (1.0 / 3) * (p1 - p0_5) + p0_5;

  // Area
  float a1 = (p0_5 - minVal) * h;
  float a2 = (1.0 / 2) * (p1 - p0_5) * h;

  // Overall Centroid
  FuzzyCentroid fc;
  fc.area = a1 + a2;

  if (fc.area != 0) {
    fc.centroid = (a1 * u1 + a2 * u2) / fc.area;
  } else {
    fc.centroid = 0;
  }

  return fc;
}

FuzzyCentroid FuzzyLogic::lins_dmf(float h, float p[3]) {
  float maxVal = p[2];

  // Points
  float p0 = p[0];                        // Bottom Left
  float p0_5 = (p[1] - p[0]) * h + p[0];  // Top Right

  // Centroids
  float u1 = (2.0 / 3) * (p0_5 - p0) + p0;
  float u2 = (maxVal - p0_5) * h + p0_5;

  // Area
  float a1 = (1.0 / 2) * (p0_5 - p0) * h;
  float a2 = (maxVal - p0_5) * h;

  // Overall Centroid
  FuzzyCentroid fc;

  fc.area = a1 + a2;

  if (fc.area != 0) {
    fc.centroid = (a1 * u1 + a2 * u2) / fc.area;
  } else {
    fc.centroid = 0;
  }

  return fc;
}

FuzzyValue FuzzyLogic::fuzzification(float x, FuzzySet fs) {
  float res[5];
  float maxVal = 0;

  float a = 0;
  float b = 0;
  float c = 0;
  float d = 0;
  float e = 0;

  bool end = false;

  // Define Max Lambda (Arduino was having issues)
  auto max = [](float a, float b) { return a > b ? a : b; };

  // Calculate membership functions for given fs.size
  a = linz_fmf(x, fs.a);

  if (fs.size > 2) {
    b = tri_fmf(x, fs.b);
  } else if (!end) {
    b = lins_fmf(x, fs.b);
    end = true;
  }

  if (fs.size > 3) {
    c = tri_fmf(x, fs.c);
  } else if (!end) {
    c = lins_fmf(x, fs.c);
    end = true;
  }

  if (fs.size > 4) {
    d = tri_fmf(x, fs.d);
  } else if (!end) {
    d = lins_fmf(x, fs.d);
    end = true;
  }

  if (!end) {
    e = lins_fmf(x, fs.e);
  }

  // Return FuzzyValue
  return {a, b, c, d, e};
}

float FuzzyLogic::defuzzification(FuzzyResult fr, FuzzySet fs) {
  FuzzyCentroid centroids[5];

  // Calculate centroids -- NOTE ONLY WORKS for FS.size = 5 as that is the only
  // size used for movement
  centroids[0] = linz_dmf(fr.a, fs.a);
  centroids[1] = tri_dmf(fr.b, fs.b);
  centroids[2] = tri_dmf(fr.c, fs.c);
  centroids[3] = tri_dmf(fr.d, fs.d);
  centroids[4] = lins_dmf(fr.e, fs.e);

  float totalCentroid = 0;
  float totalArea = 0;

  // Calculate total centroid and area
  for (int i = 0; i < 5; i++) {
    totalCentroid += centroids[i].centroid * centroids[i].area;
    totalArea += centroids[i].area;
  }

  // Calculate final result
  if (totalArea == 0) return 0;
  float result = totalCentroid / totalArea;

  return result;
}

FuzzyOutput FuzzyLogic::aggregate(FuzzyOutput a, FuzzyOutput b) {
  FuzzyOutput res;

  // MAX
  // auto method = [](float a, float b){return a > b ? a : b;};

  // SUM
  auto method = [](float a, float b) { return a + b; };

  // AVG
  // auto method = [](float a, float b){return (a + b)/2;};

  res.x.a = method(a.x.a, b.x.a);
  res.x.b = method(a.x.b, b.x.b);
  res.x.c = method(a.x.c, b.x.c);
  res.x.d = method(a.x.d, b.x.d);
  res.x.e = method(a.x.e, b.x.e);

  res.y.a = method(a.y.a, b.y.a);
  res.y.b = method(a.y.b, b.y.b);
  res.y.c = method(a.y.c, b.y.c);
  res.y.d = method(a.y.d, b.y.d);
  res.y.e = method(a.y.e, b.y.e);

  res.w.a = method(a.w.a, b.w.a);
  res.w.b = method(a.w.b, b.w.b);
  res.w.c = method(a.w.c, b.w.c);
  res.w.d = method(a.w.d, b.w.d);
  res.w.e = method(a.w.e, b.w.e);

  return res;
}

void FuzzyLogic::control(
    // Inputs
    float FrontI, float LeftI, float RightI, float LeftFrontI,
    float RightFrontI, float gyroI, float correctionI,
    // Outputs
    float* x, float* y, float* w,
    // Status
    bool avoid, int* rot_count, bool gap) {
  // FUZZIFICATION
  FuzzyValue Front = fuzzification(FrontI, FuzzyLogic::Ultrasonic_FS);

  FuzzyValue FrontLeft = fuzzification(LeftFrontI, FuzzyLogic::LeftFront_FS);

  FuzzyValue FrontRight = fuzzification(RightFrontI, FuzzyLogic::RightFront_FS);

  FuzzyValue Left = fuzzification(LeftI, FuzzyLogic::Left_FS);

  FuzzyValue Right = fuzzification(RightI, FuzzyLogic::Right_FS);

  FuzzyValue Angle = fuzzification(gyroI, FuzzyLogic::Gyro_FS);

  FuzzyValue Gyro = fuzzification(gyroI, FuzzyLogic::Gyro_FS);

  FuzzyValue Correction = fuzzification(correctionI, FuzzyLogic::Correction_FS);

  // DEFINE NUMBER OF SENSORS
  int numSensors = 6;
  //-----------------------

  // DEFINE NUMBER OF RULES
  const int maxRules = 24;
  FuzzyOutput fO[maxRules];
  for (int j = 0; j < maxRules; j++) {
    fO[j].x = {0, 0, 0, 0, 0, 0};
    fO[j].y = {0, 0, 0, 0, 0, 0};
    fO[j].w = {0, 0, 0, 0, 0, 0};
  }

#define RULE(Xin, Yin, Win, weight) \
  {                                 \
    fO[i].x.Xin = weight;           \
    fO[i].y.Yin = weight;           \
    fO[i].w.Win = weight;           \
                                    \
    i++;                            \
  }

  int i = 0;

  auto max = [](float a, float b) { return a > b ? a : b; };
  bool frontClose = (Front.a != 0) || (Front.b != 0);
  bool frontLeftClose = (FrontLeft.a != 0) || (FrontLeft.b != 0);
  bool frontRightClose = (FrontRight.a != 0) || (FrontRight.b != 0);
  bool leftBlocked = (Left.a != 0) || (Left.b != 0);
  bool rightBlocked = (Right.a != 0) || (Right.b != 0);
  bool leftClear = !leftBlocked;
  bool rightClear = !rightBlocked;

  ///////////////////////////////////////////////////////////////////////////////////////
  /// ------------------------------------ RULES
  /// ------------------------------------ ///
  ///////////////////////////////////////////////////////////////////////////////////////

  if (avoid) {
    if (frontLeftClose && frontRightClose) {
      double weight =
          max(max(FrontLeft.a, FrontLeft.b), max(FrontRight.a, FrontRight.b));
      RULE(a, none, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    // Same-side traps: back away while nudging away from the blocked wall.
    if (frontLeftClose && leftBlocked) {
      double weight = max(max(FrontLeft.a, Left.a), max(FrontLeft.b, Left.b));
      RULE(a, e, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    if (frontRightClose && rightBlocked) {
      double weight =
          max(max(FrontRight.a, Right.a), max(FrontRight.b, Right.b));
      RULE(a, a, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    // If FrontLeft is very close and Right is not very close, strafe right
    if (FrontLeft.a != 0 && Left.b == 0 && Right.b == 0 && FrontRight.a == 0) {
      double weight = 0.9 * FrontLeft.a;
      RULE(none, e, none, weight)
    }

    // If FrontRight is very close and Left is not very close, strafe left
    if (FrontRight.a != 0 && Right.b == 0 && Left.b == 0 && FrontLeft.a == 0) {
      double weight = 0.8 * FrontRight.a;
      RULE(none, a, none, weight);
    }

    // If FrontLeft is very close and Right is not close, strafe right
    if (FrontLeft.a != 0 && Left.b == 0 && Right.a == 0 && FrontRight.a == 0) {
      double weight = 0.8 * FrontLeft.a;
      RULE(none, e, none, weight)
      if (weight > 0.9) *rot_count = 5;
    }

    // If FrontRight is very close and Left is not close, strafe left
    if (FrontRight.a != 0 && Right.b == 0 && Left.a == 0 && FrontLeft.a == 0) {
      double weight = 0.9 * FrontRight.a;
      RULE(none, a, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    // If FrontLeft is close and Right is not very close, strafe right
    if (FrontLeft.b != 0 && Left.b == 0 && Right.a == 0 && Right.b == 0 &&
        FrontRight.a == 0) {
      double weight = FrontLeft.b;
      RULE(none, d, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    // If FrontRight is close and Left is not very close, strafe left
    if (FrontRight.b != 0 && Right.b == 0 && Left.a == 0 && Left.b == 0 &&
        FrontLeft.a == 0) {
      double weight = FrontRight.b;
      RULE(none, b, none, weight);
      if (weight > 0.9) *rot_count = 5;
    }

    if (!gap) {
      // Strafe if front
      if (frontClose) {
        double weight = 0.7 * max(Front.a, Front.b);

        if (frontLeftClose && !frontRightClose && rightClear) {
          RULE(none, e, none, weight);
        } else if (frontRightClose && !frontLeftClose && leftClear) {
          RULE(none, a, none, weight);
        } else if (leftClear && !frontLeftClose) {
          RULE(none, a, none, weight);
        } else if (rightClear && !frontRightClose) {
          RULE(none, e, none, weight);
        }
      }

      // Rotation
      if (((FrontRight.a != 0) || (FrontRight.b != 0)) &&
          ((Left.a != 0) || (Left.b != 0))) {
        double weight =
            max(max(FrontRight.a, Left.a), max(FrontRight.b, Left.b));
        if (!gap) {
          RULE(a, none, e, weight);
        } else {
          RULE(none, none, e, weight);
        }
        if (weight > 0.9) *rot_count = 12;
      }

      if (((FrontLeft.a != 0) || (FrontLeft.b != 0)) &&
          ((Right.a != 0) || (Right.b != 0))) {
        double weight =
            max(max(FrontLeft.a, Right.a), max(FrontLeft.b, Right.b));
        if (!gap) {
          RULE(a, none, a, weight);
        } else {
          RULE(none, none, a, weight);
        }
        if (weight > 0.9) *rot_count = 12;
      }
    }
  }

  if (!gap) {
    // If Front is far or veryfar and FrontLeft is > veryclose and FrontRight is
    // > veryclose, go forward
    if ((Front.d != 0) && !frontLeftClose && !frontRightClose) {
      double weight = Front.d;
      RULE(e, none, none, weight);
    }

    // similar to above
    if ((Front.c != 0) && !frontLeftClose && !frontRightClose) {
      double weight = Front.c;
      RULE(d, none, none, weight);
    }
  }

  if (*rot_count <= 0) {
    // If PT is to the left, turn right
    if (Correction.a != 0) {
      double weight = Correction.a;
      RULE(none, none, d, weight);
    }

    // If PT is to the right, turn left
    if (Correction.c != 0) {
      double weight = Correction.c;
      RULE(none, none, b, weight);
    }
  }

  if (gap) {
    if ((Left.a != 0 || Left.b != 0) && (Right.a == 0 || Right.b == 0)) {
      double weight = max(Left.a, Left.b);
      RULE(none, d, none, weight);
    }

    if ((Right.a != 0 || Right.b != 0) && (Left.a == 0 || Left.b == 0)) {
      double weight = max(Right.a, Right.b);
      RULE(none, b, none, weight);
    }
  }

  *rot_count = *rot_count - 1;

  if (!avoid) {
    if (Front.b != 0) {
      double weight = Front.b;
      RULE(d, none, none, weight);
    }
  }

  ///////////////////////////////////////////////////////////////////////////////////////

  // Aggregate all rules
  FuzzyOutput fOt = fO[0];
  for (int j = 1; j < i; j++) {
    fOt = aggregate(fOt, fO[j]);
  }

  float scalar = 1.0;

  // DEFUZZIFICATION and output
  *x = (float)(scalar * defuzzification(fOt.x, FuzzyLogic::X_FS));
  *y = (float)(scalar * defuzzification(fOt.y, FuzzyLogic::Y_FS));
  *w = (float)(0.2 * defuzzification(fOt.w, FuzzyLogic::W_FS));

  // Integrate lateral hysteresis in the fuzzy layer to prevent left-right
  // chatter while still responding when a side suddenly becomes unsafe.
  static float yLockDir = 0.0f;
  static int yLockCycles = 0;
  const int kYLockHoldCycles = 8;

  // Use actual side-wall pressure for lateral stabilization. Front-corner fire
  // sensing is still used by the rules above, but it should not be treated as
  // a side obstacle for hysteresis, otherwise the robot orbits around an open
  // fire instead of approaching it straight.
  float leftWallPressure = max(Left.a, Left.b);
  float rightWallPressure = max(Right.a, Right.b);
  float frontCornerPressure =
      max(max(FrontLeft.a, FrontLeft.b), max(FrontRight.a, FrontRight.b));
  float wallPressure = max(leftWallPressure, rightWallPressure);
  float wallImbalance = fabs(rightWallPressure - leftWallPressure);
  float desiredDir = Helpers::sgn(*y);

  if (avoid && !gap && frontClose && wallPressure > 0.20f) {
    if (yLockDir == 0.0f && desiredDir != 0.0f) {
      yLockDir = desiredDir;
      yLockCycles = kYLockHoldCycles;
    }

    // If current strafe heads into a newly blocked side, immediately flip.
    if (yLockDir > 0.0f && rightWallPressure > 0.30f &&
        leftWallPressure < rightWallPressure) {
      yLockDir = -1.0f;
      yLockCycles = kYLockHoldCycles;
    } else if (yLockDir < 0.0f && leftWallPressure > 0.30f &&
               rightWallPressure < leftWallPressure) {
      yLockDir = 1.0f;
      yLockCycles = kYLockHoldCycles;
    }

    if (yLockDir != 0.0f) {
      if (yLockCycles > 0) {
        yLockCycles--;
      } else {
        if (desiredDir != 0.0f && desiredDir != yLockDir &&
            wallImbalance > 0.18f) {
          yLockDir = desiredDir;
        }
        yLockCycles = kYLockHoldCycles;
      }

      // Keep fuzzy output magnitude but stabilize direction.
      if (fabs(*y) > 0.05f) {
        *y = fabs(*y) * yLockDir;
      }
    }
  } else {
    yLockDir = 0.0f;
    yLockCycles = 0;
  }

  // In open-space fire approach, keep the robot from circling by zeroing
  // lateral motion when only the front-corner fire sensors are active.
  if (avoid && !gap && frontClose && wallPressure < 0.12f &&
      frontCornerPressure > 0.20f) {
    *y = 0.0f;
    *w = constrain(*w, -0.12f, 0.12f);
  }
}

char getMax(FuzzyValue fv) {
  float max = 0;
  char type = 'a';
  if (fv.a > max) {
    max = fv.a;
    type = 'a';
  }
  if (fv.b > max) {
    max = fv.b;
    type = 'b';
  }
  if (fv.c > max) {
    max = fv.c;
    type = 'c';
  }
  if (fv.d > max) {
    max = fv.d;
    type = 'd';
  }
  if (fv.e > max) {
    max = fv.e;
    type = 'e';
  }
  return type;
}
