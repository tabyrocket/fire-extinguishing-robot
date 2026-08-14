#include <Arduino.h>

#include "./src/Robot/Robot.h"
#include "./src/Misc/Helpers/Helpers.h"

Robot robot;

void setup(){
    Helpers::serialBegin();
    Helpers::serialPrint("Starting Robot...", 1);
    robot = Robot();
    robot.enable();
    Helpers::serialPrint("Started Robot", 1);
}

// To try and simplify the program, we can directly update the FSM from here
void loop(){
    // testing();	
    while(1){
        robot.finiteStateMachine->update(); 
        delay(1);
    }
}

void testing(){
    // Phototransistor testing
    robot.fanServo->set_angle(0);
    delay(1000);
    robot.fanServo->disable();

    // int adcVal1_p = 0;
    // int adcVal2_p = 0;
    // int adcVal3_p = 0;
    // int adcVal4_p = 0;
    double r = 0;
    double l = 0;
    while (1){
        robot.irSensors->read(-1);
        
        r = Helpers::movingAverage(robot.irSensors->sensors[3].distance, r, 3, 1);
        l = Helpers::movingAverage(robot.irSensors->sensors[1].distance, l, 3, 1);
        
        Helpers::telePrint("Right", r);
        Helpers::telePrint("Left", l);
        
        delay(25);

        // robot.phototransistors->read(-1);
        // sensors order: A2, A3, A6, A5
        // int adcVal1 = robot.phototransistors->sensors[0].value;
        // int adcVal2 = robot.phototransistors->sensors[1].value;
        // int adcVal3 = robot.phototransistors->sensors[2].value;
        // int adcVal4 = robot.phototransistors->sensors[3].value;

        // Print Unfiltered
        // Helpers::serialPrint(">Far Left PT: ");
        // Helpers::serialPrint(adcVal1, 1);
        // Helpers::serialPrint(">Far Right PT: ");
        // Helpers::serialPrint(adcVal4, 1);
        // Helpers::serialPrint(">Left Centre PT: ");
        // Helpers::serialPrint(adcVal2, 1);
        // Helpers::serialPrint(">Right Centre PT: ");
        // Helpers::serialPrint(adcVal3, 1);

        // filter the PT values
        // adcVal1 = Helpers::movingAverage(adcVal1, adcVal1_p, 3, 1);
        // adcVal2 = Helpers::movingAverage(adcVal2, adcVal2_p, 3, 1);
        // adcVal3 = Helpers::movingAverage(adcVal3, adcVal3_p, 3, 1);
        // adcVal4 = Helpers::movingAverage(adcVal4, adcVal4_p, 3, 1);

        // adcVal1_p = adcVal1;
        // adcVal2_p = adcVal2;
        // adcVal3_p = adcVal3;
        // adcVal4_p = adcVal4;

        // print filtered
        // Helpers::serialPrint(">FAR LEFT: ");
        // Helpers::serialPrint(adcVal1, 1);
        // Helpers::serialPrint(">FILTERED FAR RIGHT: ");
        // Helpers::serialPrint(adcVal4, 1);
        // Helpers::serialPrint(">CENTRE LEFT: ");
        // Helpers::serialPrint(adcVal2, 1);
        // Helpers::serialPrint(">CENTRE RIGHT: ");
        // Helpers::serialPrint(adcVal3, 1);
        // Helpers::serialPrint(">Difference from right to left: ");
        // Helpers::serialPrint(adcVal3-adcVal2, 1);
        // Helpers::serialPrint(">Sum of right and left: ");
        // Helpers::serialPrint(adcVal3+adcVal2, 1);
        // Helpers::serialPrint(">Difference of right and left: ");
        // Helpers::serialPrint(adcVal3-adcVal2, 1);
        // delay(100);
    }

    // Aim testing
    // reset angle by placing mount where 0 angle is

    // sweep
    // robot.fanServo->set_angle(0);
    // delay(1000);
    // robot.fanServo->disable();
    // robot.fanServo->set_angle(75);
    // delay(1000);
    // robot.fanServo->set_angle(-75);
    // delay(1000);    
    // for (int i = -75; i < 75; i++) {
    //   robot.fanServo->set_angle(i);
    //   delay(10);
    // }


    // GYRO TESTING
    // robot.gyro->calibrate();
    // robot.gyro->reset();
    // while (1) {
    //     robot.gyro->update();

    //     Helpers::serialPrint(">Angle: ");
    //     Helpers::serialPrint(robot.gyro->currentAngle, 1);
    //     Helpers::serialPrint(">Control Angle: ");
    //     Helpers::serialPrint(robot.control->gyro->currentAngle, 1);
    //     delay(1);
    // }


    // turning testing
    // robot.control.moveDirection(0,0,1,200);
    // delay(1000);

    // robot.gyro->calibrate();
    // robot.gyro->reset();
    // robot.control->controlAngle(180);

    // time for spinning experimenting
    // robot.control->moveDirection(0,0,1,250);
    // long maxStart = millis();
    // long currentTime = maxStart;
    // long requiredTime = 4000; // time to check for maximum PT value
    // while (currentTime - maxStart < requiredTime) {
    //     currentTime = millis();
    //     delay(1);
    // }
    // robot.control->moveDirection(0,0,1,0);


    //  aim testing
    // robot.gyro->calibrate();
    // robot.gyro->reset();
    // robot.control->aim();

    // // print PT values
    // while (1){
    //     robot.phototransistors->read(-1);
    //     // sensors order: A2, A3, A6, A5
    //     int adcVal1 = robot.phototransistors->sensors[0].value;
    //     int adcVal2 = robot.phototransistors->sensors[1].value;
    //     int adcVal3 = robot.phototransistors->sensors[2].value;
    //     int adcVal4 = robot.phototransistors->sensors[3].value;
    //     Helpers::serialPrint(">Left Centre PT: ");
    //     Helpers::serialPrint(adcVal2, 1);
    //     Helpers::serialPrint(">Right Centre PT: ");
    //     Helpers::serialPrint(adcVal3, 1);
    //     Helpers::serialPrint(">Difference from right to left: ");
    //     Helpers::serialPrint(adcVal3-adcVal2, 1);
    //     Helpers::serialPrint(">Sum of right and left: ");
    //     Helpers::serialPrint(adcVal3+adcVal2, 1);
    //     delay(10);
    // }

    // Dhruv testing
    // sweepPT();

    while(1){
        int i = 1;
    }

}

void sweepPT() {
    int sweepAngle = 75; 
  int offsetAngle = 40; // side PT angle offset
  int edgeThreshold = 5; // angle from edge to ignore maximum PT value found

  // set servo to leftmost position
  robot.fanServo->set_angle(-sweepAngle);
  delay(1000);

  // filtering stuff
  int adcVal1;
  int adcVal2;
  int adcVal3;
  int adcVal4;
  int adcVal1_p = 0;
  int adcVal2_p = 0;
  int adcVal3_p = 0;
  int adcVal4_p = 0;

  for (int i = 0; i < 10; i++){
        robot.phototransistors->read(-1);
        // sensors order: A2, A3, A6, A5
        int adcVal1 = robot.phototransistors->sensors[0].value;
        int adcVal2 = robot.phototransistors->sensors[1].value;
        int adcVal3 = robot.phototransistors->sensors[2].value;
        int adcVal4 = robot.phototransistors->sensors[3].value;

        // filter the PT values
        adcVal1 = Helpers::movingAverage(adcVal1, adcVal1_p, 3, 1);
        adcVal2 = Helpers::movingAverage(adcVal2, adcVal2_p, 3, 1);
        adcVal3 = Helpers::movingAverage(adcVal3, adcVal3_p, 3, 1);
        adcVal4 = Helpers::movingAverage(adcVal4, adcVal4_p, 3, 1);

        adcVal1_p = adcVal1;
        adcVal2_p = adcVal2;
        adcVal3_p = adcVal3;
        adcVal4_p = adcVal4;

        delay(5);
    }

    int PT_values[sweepAngle*2+1];
    int PT_values_l[sweepAngle*2+1];
    int PT_values_r[sweepAngle*2+1];
    int fireFound = 0;
    int max_angle = 0;
    int max_angle_l = 0;
    int max_angle_r = 0;

    for (int i = -sweepAngle; i <= sweepAngle; i++) {
      robot.fanServo->set_angle(i);
      delay(25);

      robot.phototransistors->read(-1);
    // sensors order: A2, A3, A6, A5
      int adcVal1 = robot.phototransistors->sensors[0].value;
      int adcVal2 = robot.phototransistors->sensors[1].value;
      int adcVal3 = robot.phototransistors->sensors[2].value;
      int adcVal4 = robot.phototransistors->sensors[3].value;

      // filter the PT values
      adcVal1 = Helpers::movingAverage(adcVal1, adcVal1_p, 3, 1);
      adcVal2 = Helpers::movingAverage(adcVal2, adcVal2_p, 3, 1);
      adcVal3 = Helpers::movingAverage(adcVal3, adcVal3_p, 3, 1);
      adcVal4 = Helpers::movingAverage(adcVal4, adcVal4_p, 3, 1);

      adcVal1_p = adcVal1;  // fl 
      adcVal2_p = adcVal2;  // cl
      adcVal3_p = adcVal3;  // cr
      adcVal4_p = adcVal4;  // fr

      PT_values[i + sweepAngle] = adcVal2_p + adcVal3_p;
      PT_values_l[i + sweepAngle] = adcVal1_p;
      PT_values_r[i + sweepAngle] = adcVal4_p;

      // TESTING
      // print the PT value for each angle
      Helpers::serialPrint(">Angle: ");
      Helpers::serialPrint(i, 1);
      Helpers::serialPrint(">L: ");
      Helpers::serialPrint(adcVal2, 1);
      Helpers::serialPrint(">R: ");
      Helpers::serialPrint(adcVal3, 1);
      Helpers::serialPrint(">R+L: ");
      Helpers::serialPrint(adcVal3 + adcVal2, 1);
      Helpers::serialPrint(">R-L: ");
      Helpers::serialPrint(adcVal3 - adcVal2, 1);
      Helpers::serialPrint(">FL: ");
      Helpers::serialPrint(adcVal1, 1);
      Helpers::serialPrint(">FR: ");
      Helpers::serialPrint(adcVal4, 1);
    }
}