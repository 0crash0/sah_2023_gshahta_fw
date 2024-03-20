/*
  можно 17 16 уарт
  там где wifi uart2

  #define AUX4_17_PIN                           17  LCD_RS1
  #define AUX4_18_PIN                           16     LCD_EN1
  LCD_EN1

  interrupts for communication
  Mega, Mega2560, MegaADK
  2, 3, 18, 19, 20, 21 (pins 20 & 21 are not available to use for interrupts while they are used for I2C communication)
  Mega
  INT.0   INT.1   INT.2   INT.3   INT.4   INT.5
  2       3       21      20      19      18
*/
///////////////////////////////////////////////////////////////////////////// i2c
//#include <Wire.h>

#define PAYLOAD_SIZE 10
////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////// drivers

#define DRIVER_STEP_TIME 10  // меняем задержку на 10 мкс

#define GS_FAST_PROFILE 10
#include "GyverStepper2.h"

#define pos_top 14000   /// не уменьшенная высота = 60000 = 37,5мм (ошибка 14000=10мм

#define pos_down 0

#define power_OFF_while_STANDING true

#define randomizing_delay 2000

//uint32_t tar = 80000;
bool dir = 1;

#define REQUIRE_MEGA2560
#include "macros.h"
#include "pins_MKS_GEN_13.h"


#define MAX_SPEED 15000
#define ACCEL 7000
#define HOME_SPEED -10000

bool dirX, dirY, dirZ, dirE0, dirE1;

enum class axis {X, Y, Z, E0, E1, ALL};

enum class state {mooving, stay};

enum class position {unknown, down, top};

position nowLocation[5] = {position::unknown, position::unknown, position::unknown, position::unknown, position::unknown};
position nextLocation[5] = {position::down, position::down, position::down, position::down, position::down};

//position setLocation[5] =  {position::unknown, position::unknown, position::unknown, position::unknown, position::unknown};

//state motorsStatus[5] = {state::stay, state::stay, state::stay, state::stay, state::stay};


bool AllJobsDone = false;

int rndMotorsToMoove = 0;
bool mtrToMove[5] = {false, false, false, false, false};


GStepper2 <STEPPER2WIRE> stepperX(2048, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperY(2048, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperZ(2048, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperE0(2048, E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperE1(2048, E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN);


//
// AUX1    5V  GND D2  D1
//          2   4   6   8
//          1   3   5   7
//         5V  GND A3  A4
// AUX2    GND A9 D40 D42 A11
//          2   4   6   8  10
//          1   3   5   7   9
//         VCC A5 A10 D44 A12
#include <SoftwareSerial.h>
#include <TMC2208Stepper.h>
#define R_SENSE 0.11f
TMC2208Stepper driver0 = TMC2208Stepper( AUX1_06_PIN,AUX1_05_PIN); // Software serial
TMC2208Stepper driver1 = TMC2208Stepper( AUX2_04_PIN,AUX2_03_PIN); // Software serial
TMC2208Stepper driver2 = TMC2208Stepper( AUX2_06_PIN,AUX2_05_PIN); // Software serial
TMC2208Stepper driver3 = TMC2208Stepper( AUX2_08_PIN,AUX2_07_PIN); // Software serial
TMC2208Stepper driver4 = TMC2208Stepper( AUX2_10_PIN,AUX2_09_PIN); // Software serial



void tmc2208_init_in_my() {
  uint32_t data = 0;
  uint32_t drv_status;
  driver0.beginSerial(115200);
  driver0.push();
  //driver0.begin();             // Initiate pins and registeries
  driver0.pdn_disable(1);  
  driver0.I_scale_analog(0);
  driver0.rms_current(700);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  //driver0.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver0.pwm_autoscale(1);
  driver0.microsteps(16);
driver0.DRV_STATUS(&drv_status);
Serial.print("drv_status=");
Serial.println(drv_status, HEX);
  Serial.print("current set to:");
  Serial.println(driver0.rms_current());

driver1.beginSerial(115200);
  //driver1.begin();             // Initiate pins and registeries
  driver1.pdn_disable(1);  
  driver1.I_scale_analog(0);
  driver1.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  //driver1.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver1.pwm_autoscale(1);
  driver1.microsteps(16);
driver1.DRV_STATUS(&drv_status);
Serial.print("drv_status=");
Serial.println(drv_status, HEX);
  Serial.print("current set to:");
  Serial.println(driver1.rms_current());

driver2.beginSerial(115200);
  //driver2.begin();             // Initiate pins and registeries
  driver2.pdn_disable(1); 
  driver2.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver2.I_scale_analog(0);
  //driver2.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver2.pwm_autoscale(1);
  driver2.microsteps(16);
driver2.DRV_STATUS(&drv_status);
Serial.print("drv_status=");
Serial.println(drv_status, HEX);
  Serial.print("current set to:");
  Serial.println(driver2.rms_current());
driver3.beginSerial(115200);
  //driver3.begin();             // Initiate pins and registeries
  driver3.pdn_disable(1);  
  driver3.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  driver3.I_scale_analog(0);
  //driver3.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver3.pwm_autoscale(1);
  driver3.microsteps(16);
driver3.DRV_STATUS(&drv_status);
Serial.print("drv_status=");
Serial.println(drv_status, HEX);
  Serial.print("current set to:");
  Serial.println(driver3.rms_current());
driver4.beginSerial(115200);
  //driver4.begin();             // Initiate pins and registeries
  driver4.pdn_disable(1);  
  driver4.I_scale_analog(0);
  driver4.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
  //driver4.en_pwm_mode(1);      // Enable extremely quiet stepping
  driver4.pwm_autoscale(1);
  driver4.microsteps(16);
driver4.DRV_STATUS(&drv_status);
Serial.print("drv_status=");
Serial.println(drv_status, HEX);
  Serial.print("current set to:");
  Serial.println(driver4.rms_current());

  delay(10000);
}

void setup() {
  pinMode(X_MIN_PIN, INPUT);// pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Y_MAX_PIN, INPUT);
  pinMode(Z_MIN_PIN, INPUT);
  pinMode(Z_MAX_PIN, INPUT);

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_CS_PIN, OUTPUT);
  pinMode(Y_CS_PIN, OUTPUT);
  pinMode(Z_CS_PIN, OUTPUT);
  pinMode(E0_CS_PIN, OUTPUT);
  pinMode(E1_CS_PIN, OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Z_ENABLE_PIN, OUTPUT);
  pinMode(E0_ENABLE_PIN, OUTPUT);
  pinMode(E1_ENABLE_PIN, OUTPUT);




  Serial.begin(9600);
  Serial.println("Hello...");
  stepperX.setMaxSpeed(MAX_SPEED);     // скорость движения к цели
  stepperX.setAcceleration(ACCEL); // ускорение

  stepperY.setMaxSpeed(MAX_SPEED);     // скорость движения к цели
  stepperY.setAcceleration(ACCEL); // ускорение

  stepperZ.setMaxSpeed(MAX_SPEED);     // скорость движения к цели
  stepperZ.setAcceleration(ACCEL); // ускорение

  stepperE0.setMaxSpeed(MAX_SPEED);     // скорость движения к цели
  stepperE0.setAcceleration(ACCEL); // ускорение

  stepperE1.setMaxSpeed(MAX_SPEED);     // скорость движения к цели
  stepperE1.setAcceleration(ACCEL); // ускорение

tmc2208_init_in_my();

  homing(axis::ALL);

}


void loop() {
  //X
  /*
  */



  if (stepperX.getStatus() == 0) {
    //Serial.println("status = 0");

    if (stepperX.pos == pos_top) {

      nowLocation[0] = position::top;
    }
    if (stepperX.pos == pos_down) {
      nowLocation[0] = position::down;
    }
    if (nowLocation[0] != nextLocation[0]) {
      if (power_OFF_while_STANDING) {
        stepperX.enable();
      }

      if (nextLocation[0] == position::top) {
        stepperX.setTarget(pos_top);
        nowLocation[0] = position::unknown;
        Serial.println("..................doing to top");
      }
      if (nextLocation[0] == position::down) {//else {
        stepperX.setTarget(pos_down);
        nowLocation[0] = position::unknown;
        //homing(axis::X);
        Serial.println("targetX down");

        Serial.println("..................doing to down");
      }
    } else {
      //Serial.println(stepperX.pos);
      // AllJobsDone = true;
      if (power_OFF_while_STANDING) {
        stepperX.disable();
      }

      mtrToMove[0] = true;
    }
  } else {
    mtrToMove[0] = false;
    //AllJobsDone = false;
  }



  if (stepperY.getStatus() == 0) {
    //Serial.println("status = 0");
    if (stepperY.pos == pos_top) {
      nowLocation[1] = position::top;
    }
    if (stepperY.pos == pos_down) {
      nowLocation[1] = position::down;
    }
    if (nowLocation[1] != nextLocation[1]) {
      if (power_OFF_while_STANDING) {
        stepperY.enable();
      }
      if (nextLocation[1] == position::top) {
        stepperY.setTarget(pos_top);
        nowLocation[1] = position::unknown;
        Serial.println("..................doing to top");
      }
      if (nextLocation[1] == position::down) {//else {
        stepperY.setTarget(pos_down);
        nowLocation[1] = position::unknown;
        //homing(axis::X);
        Serial.println("targetY down");

        Serial.println("..................doing to down");
      }
    } else {
      //Serial.println(stepperX.pos);
      // AllJobsDone = true;
      if (power_OFF_while_STANDING) {
        stepperY.disable();
      }
      mtrToMove[1] = true;
    }
  } else {
    mtrToMove[1] = false;
    //AllJobsDone = false;
  }


  if (stepperZ.getStatus() == 0) {
    //Serial.println("status = 0");
    if (stepperZ.pos == pos_top) {
      nowLocation[2] = position::top;
    }
    if (stepperZ.pos == pos_down) {
      nowLocation[2] = position::down;
    }
    if (nowLocation[2] != nextLocation[2]) {
      if (power_OFF_while_STANDING) {
        stepperZ.enable();
      }
      if (nextLocation[2] == position::top) {
        stepperZ.setTarget(pos_top);
        nowLocation[2] = position::unknown;
        Serial.println("..................doing to top");
      }
      if (nextLocation[2] == position::down) {//else {
        stepperZ.setTarget(pos_down);
        nowLocation[2] = position::unknown;
        //homing(axis::X);
        Serial.println("targetZ down");

        Serial.println("..................doing to down");
      }
    } else {
      //Serial.println(stepperX.pos);
      // AllJobsDone = true;
      if (power_OFF_while_STANDING) {
        stepperZ.disable();
      }
      mtrToMove[2] = true;
    }
  } else {
    mtrToMove[2] = false;
    //AllJobsDone = false;
  }

  if (stepperE0.getStatus() == 0) {
    //Serial.println("status = 0");
    if (stepperE0.pos == pos_top) {
      nowLocation[3] = position::top;
    }
    if (stepperE0.pos == pos_down) {
      nowLocation[3] = position::down;
    }
    if (nowLocation[3] != nextLocation[3]) {
      if (power_OFF_while_STANDING) {
        stepperE0.enable();
      }
      if (nextLocation[3] == position::top) {
        stepperE0.setTarget(pos_top);
        nowLocation[3] = position::unknown;
        Serial.println("..................doing to top");
      }
      if (nextLocation[3] == position::down) {//else {
        stepperE0.setTarget(pos_down);
        nowLocation[3] = position::unknown;
        //homing(axis::X);
        Serial.println("targetE0 down");

        Serial.println("..................doing to down");
      }
    } else {
      //Serial.println(stepperX.pos);
      // AllJobsDone = true;
      if (power_OFF_while_STANDING) {
        stepperE0.disable();
      }
      mtrToMove[3] = true;
    }
  } else {
    mtrToMove[3] = false;
    //AllJobsDone = false;
  }



  if (stepperE1.getStatus() == 0) {
    //Serial.println("status = 0");
    if (stepperE1.pos == pos_top) {
      nowLocation[4] = position::top;
    }
    if (stepperE1.pos == pos_down) {
      nowLocation[4] = position::down;
    }
    if (nowLocation[4] != nextLocation[4]) {
      if (power_OFF_while_STANDING) {
        stepperE1.enable();
      }
      if (nextLocation[4] == position::top) {
        stepperE1.setTarget(pos_top);
        nowLocation[4] = position::unknown;
        Serial.println("..................doing to top");
      }
      if (nextLocation[4] == position::down) {//else {
        stepperE1.setTarget(pos_down);
        nowLocation[4] = position::unknown;
        //homing(axis::X);
        Serial.println("targetE1 down");

        Serial.println("..................doing to down");
      }
    } else {
      //Serial.println(stepperX.pos);
      // AllJobsDone = true;
      if (power_OFF_while_STANDING) {
        stepperE1.disable();
      }
      mtrToMove[4] = true;
    }
  } else {
    mtrToMove[4] = false;
    //AllJobsDone = false;
  }

  if (stepperX.getStatus() == 0 && stepperY.getStatus() == 0 && stepperZ.getStatus() == 0 && stepperE0.getStatus() == 0 && stepperE1.getStatus() == 0) {
    if (nowLocation[0] == nextLocation[0] && nowLocation[1] == nextLocation[1] && nowLocation[2] == nextLocation[2] &&  nowLocation[3] == nextLocation[3] &&  nowLocation[4] == nextLocation[4]) {
      AllJobsDone = true;
    }
    else {
      AllJobsDone = false;
    }
  }
  /* if (nowLocation[0] == position::down && nextLocation[0] == position::top) {
     nowLocation[0] = position::top;
    }*/


  /*
    stepperX.tick();   // мотор асинхронно крутится тут
    stepperY.tick();
    stepperZ.tick();
    stepperE0.tick();
    stepperE1.tick();
  */



  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= randomizing_delay) {
    if (AllJobsDone) {
      Serial.println("AllJobsDone");
    }
    else {
      Serial.println("NOT AllJobsDone");
    }
    tmr = millis();

    //if (AllJobsDone == true) { /////// / // / / / // // // / ALL TIME RANDOMIZING
    Serial.println("Rndoming");
    //rndMotorsToMoove=random(0,5);
    for (int i = 0; i < 5; i++) {
      if (mtrToMove[i]) {
        rndMotorsToMoove = random(4);
        Serial.println(rndMotorsToMoove);
        if (rndMotorsToMoove == 1) {
          Serial.println("GO");
          if (nextLocation[i] == position::top) {
            nextLocation[i] = position::down;
          }
          else {
            nextLocation[i] = position::top;
          }
        }
      }
    }
    AllJobsDone = false;
    //}
    /*
      Serial.print("position:" );
      Serial.println( stepperX.pos);
      if (nowLocation[0] == position::top) {
       Serial.print(" now top ");
      }
      if (nowLocation[0] == position::down) {
       Serial.print(" now down ");
      }
      if (nextLocation[0] == position::top) {
       Serial.println(" to top ");
      }
      if (nextLocation[0] == position::down) {
       Serial.println(" to down ");
      }*/
  }

}

void homing(axis homer) {
  Serial.println("Homing.....");
  if (homer == axis::X || homer == axis::ALL) {
    Serial.println("Homing X");

    if (digitalRead(X_MIN_PIN)) {       // если концевик X не нажат
      stepperX.setSpeed(HOME_SPEED);       // ось Х, -10 шаг/сек
      //stepperX.setTarget(800000);
      while (digitalRead(X_MIN_PIN)) {  // пока кнопка не нажата
        stepperX.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperX.brake();                // тормозим, приехали
    }
    stepperX.reset();    // сбрасываем координаты в 0
    nowLocation[0] = position::down;
    Serial.println("X Home!");
  }
  if (homer == axis::Y || homer == axis::ALL) {
    Serial.println("Homing Y");
    stepperY.reset();    // сбрасываем координаты в 0
    if (digitalRead(X_MAX_PIN)) {       // если концевик X не нажат
      stepperY.setSpeed(HOME_SPEED);       // ось Х, -10 шаг/сек
      while (digitalRead(X_MAX_PIN)) {  // пока кнопка не нажата
        stepperY.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperY.brake();                // тормозим, приехали
    }
    stepperY.reset();    // сбрасываем координаты в 0
    nowLocation[1] = position::down;
    Serial.println("Y Home!");
  }
  if (homer == axis::Z || homer == axis::ALL) {

    Serial.println("Homing Z");
    //stepperZ.setTarget(tar);
    stepperZ.reset();    // сбрасываем координаты в 0
    if (digitalRead(Y_MIN_PIN)) {       // если концевик X не нажат
      stepperZ.setSpeed(HOME_SPEED);       // ось Х, -10 шаг/сек
      while (digitalRead(Y_MIN_PIN)) {  // пока кнопка не нажата
        stepperZ.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperZ.brake();                // тормозим, приехали
    }
    stepperZ.reset();    // сбрасываем координаты в 0
    nowLocation[2] = position::down;
    Serial.println("Z Home!");
  }
  if (homer == axis::E0 || homer == axis::ALL) {
    Serial.println("Homing E0");
    stepperE0.reset();    // сбрасываем координаты в 0
    if (digitalRead(Y_MAX_PIN)) {       // если концевик X не нажат
      stepperE0.setSpeed(HOME_SPEED);       // ось Х, -10 шаг/сек
      while (digitalRead(Y_MAX_PIN)) {  // пока кнопка не нажата
        stepperE0.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperE0.brake();                // тормозим, приехали
    }
    stepperE0.reset();    // сбрасываем координаты в 0
    nowLocation[3] = position::down;
    Serial.println("E0 Home!");
  }
  if (homer == axis::E1 || homer == axis::ALL) {
    Serial.println("Homing E1");
    stepperE1.reset();    // сбрасываем координаты в 0
    if (digitalRead(Z_MIN_PIN)) {       // если концевик X не нажат
      stepperE1.setSpeed(HOME_SPEED);       // ось Х, -10 шаг/сек
      while (digitalRead(Z_MIN_PIN)) {  // пока кнопка не нажата
        stepperE1.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperE1.brake();                // тормозим, приехали
    }
    stepperE1.reset();    // сбрасываем координаты в 0
    nowLocation[4] = position::down;
    Serial.println("E1 Home!");
  }

  Serial.println("All axes at home");
  AllJobsDone = true;
}
