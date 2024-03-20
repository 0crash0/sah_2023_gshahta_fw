#include <AccelStepper.h>
#define REQUIRE_MEGA2560
#include "macros.h"
#include "pins_MKS_GEN_13.h"


#define STEP_PER_MM 50000

#define MOVEMENT_DELAY 10000


#define MAX_SPEED_X 10000 * STEP_PER_MM
#define MAX_ACCEL_X 50 * STEP_PER_MM
#define MIN_SPEED_X 10 * STEP_PER_MM
#define MIN_ACCEL_X 2 * STEP_PER_MM
#define HOMING_SPEED_X 10000 * STEP_PER_MM
#define HOMING_ACCEL_X 50 * STEP_PER_MM

#define MAX_SPEED_Y 1000 * STEP_PER_MM
#define MAX_ACCEL_Y 50 * STEP_PER_MM
#define MIN_SPEED_Y 20 * STEP_PER_MM
#define MIN_ACCEL_Y 5 * STEP_PER_MM
#define HOMING_SPEED_Y 20 * STEP_PER_MM
#define HOMING_ACCEL_Y 5 * STEP_PER_MM

#define HOMING_SLOW_DISTANCE 5000





AccelStepper stepperX(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper stepperY(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);

void setup() {
  pinMode(X_MIN_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_CS_PIN, OUTPUT);


  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_CS_PIN, OUTPUT);

stepperX.setPinsInverted(true,false,false);

  homing();
//stepperX.runToNewPosition(-94000);

}

int go=1;  //0 down  1 up
int changed=0;
int btn=1;

void loop() {
   if(digitalRead(X_MIN_PIN) != btn) {
      btn=digitalRead(X_MIN_PIN);
      changed=1;
   }
   
   if(go == 0 && changed == 1) {
      stepperX.runToNewPosition(50000);
      go=1;
      
      changed=0;
   }
   if(go == 1 && changed == 1) {
      stepperX.runToNewPosition(-50000);
      go=0;
      changed=0;
   }
  delay(500);
  //stepperX.run();
}


void homing() {

  stepperX.setMaxSpeed(HOMING_SPEED_X);
  stepperX.setAcceleration(HOMING_ACCEL_X);
  //stepperX.moveTo(-MAX_X*STEP_PER_MM);
  stepperX.moveTo(-1000000000);
  while (!digitalRead(X_MIN_PIN)) {
    stepperX.run();
  }
  stepperX.stop();
  stepperX.setCurrentPosition(0);

  stepperX.setMaxSpeed(MIN_SPEED_X);
  stepperX.setAcceleration(MIN_ACCEL_X);
  stepperX.moveTo(HOMING_SLOW_DISTANCE);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }
  //stepperX.stop();
  //stepperX.setCurrentPosition(0);

  stepperX.setMaxSpeed(MIN_SPEED_X);
  stepperX.setAcceleration(MIN_ACCEL_X);
  stepperX.moveTo(-HOMING_SLOW_DISTANCE);
    while (!digitalRead(X_MIN_PIN)) {
    stepperX.run();
  }
  stepperX.stop();
  stepperX.setCurrentPosition(0);


/*

  stepperY.setMaxSpeed(HOMING_SPEED_Y);
  stepperY.setAcceleration(HOMING_ACCEL_Y);
  stepperY.moveTo(MAX_Y * STEP_PER_MM * (-1));

  do {
    stepperY.run();
  } while (digitalRead(Y_MIN_PIN));
  stepperY.stop();
  // stepperX.runToPosition();
  stepperY.setCurrentPosition(0);

  stepperY.setMaxSpeed(MIN_SPEED_Y);
  stepperY.setAcceleration(MIN_ACCEL_Y);
  stepperY.moveTo(HOMING_SLOW_DISTANCE);
  do {
    stepperY.run();
  } while (stepperY.distanceToGo() == 0);
  stepperY.stop();
  // stepperX.runToPosition();
  stepperY.setCurrentPosition(0);

  stepperY.moveTo(HOMING_SLOW_DISTANCE * (-1));
  do {
    stepperY.run();
  } while (digitalRead(Y_MIN_PIN));
  stepperY.stop();
  // stepperX.runToPosition();
  stepperY.setCurrentPosition(0);*/
}



//
/*
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            53
#endif

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            49
#endif

#define X_MIN_PIN 

#define Y_MIN_PIN   
*/
