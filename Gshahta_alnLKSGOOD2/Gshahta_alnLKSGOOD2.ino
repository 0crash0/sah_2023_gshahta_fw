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

#define TIME_TO_RESET 3600000
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

digitalWrite(AUX2_10_PIN,HIGH);
pinMode(AUX2_10_PIN,OUTPUT);
digitalWrite(AUX2_10_PIN,HIGH);

  Serial.begin(115200);
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

  homing(axis::ALL);

}
bool toRESET=false;
unsigned long tmr;
void loop() {
 

  if(stepperX.getStatus() == 0 && stepperY.getStatus() == 0 && stepperZ.getStatus() == 0 && stepperE0.getStatus() == 0 && stepperE1.getStatus() == 0){
    if (nowLocation[0] == nextLocation[0] && nowLocation[1] == nextLocation[1] && nowLocation[2] == nextLocation[2] &&  nowLocation[3] == nextLocation[3] &&  nowLocation[4] == nextLocation[4]) {
      AllJobsDone = true;
      }
      else{
        AllJobsDone = false;
      }
  }
  if(millis() >= TIME_TO_RESET){//4290000000){
    toRESET=true;
    if(AllJobsDone){
    softReset();}
  }
 
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
    if (nowLocation[0] != nextLocation[0] && !toRESET) {
      if(power_OFF_while_STANDING){
        stepperX.enable();  
      }
      
      if (nextLocation[0] == position::top) {
        stepperX.setTarget(pos_top);
        nowLocation[0] = position::unknown;
        //Serial.println("..................doing to top");
      }
      if (nextLocation[0] == position::down) {//else {
        stepperX.setTarget(pos_down);
        nowLocation[0] = position::unknown;
        //homing(axis::X);
        //Serial.println("targetX down");

        //Serial.println("..................doing to down");
      }
    }else{
      //Serial.println(stepperX.pos);
     // AllJobsDone = true;
     if(power_OFF_while_STANDING){
      stepperX.disable();
     }
      
     mtrToMove[0]=true;
    }
  }else{
    mtrToMove[0]=false;
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
    if (nowLocation[1] != nextLocation[1] && !toRESET) {
      if(power_OFF_while_STANDING){
        stepperY.enable();  
      }
      if (nextLocation[1] == position::top) {
        stepperY.setTarget(pos_top);
        nowLocation[1] = position::unknown;
        //Serial.println("..................doing to top");
      }
      if (nextLocation[1] == position::down) {//else {
        stepperY.setTarget(pos_down);
        nowLocation[1] = position::unknown;
        //homing(axis::X);
        //Serial.println("targetY down");

        //Serial.println("..................doing to down");
      }
  }else{
      //Serial.println(stepperX.pos);
     // AllJobsDone = true;
     if(power_OFF_while_STANDING){
      stepperY.disable();
     }
     mtrToMove[1]=true;
    }
  }else{
    mtrToMove[1]=false;
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
    if (nowLocation[2] != nextLocation[2] && !toRESET) {
      if(power_OFF_while_STANDING){
        stepperZ.enable();  
      }
      if (nextLocation[2] == position::top) {
        stepperZ.setTarget(pos_top);
        nowLocation[2] = position::unknown;
        //Serial.println("..................doing to top");
      }
      if (nextLocation[2] == position::down) {//else {
        stepperZ.setTarget(pos_down);
        nowLocation[2] = position::unknown;
        //homing(axis::X);
        //Serial.println("targetZ down");

        //Serial.println("..................doing to down");
      }
  }else{
      //Serial.println(stepperX.pos);
     // AllJobsDone = true;
     if(power_OFF_while_STANDING){
      stepperZ.disable();
     }
     mtrToMove[2]=true;
    }
  }else{
    mtrToMove[2]=false;
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
    if (nowLocation[3] != nextLocation[3] && !toRESET) {
      if(power_OFF_while_STANDING){
        stepperE0.enable();  
      }
      if (nextLocation[3] == position::top) {
        stepperE0.setTarget(pos_top);
        nowLocation[3] = position::unknown;
        //Serial.println("..................doing to top");
      }
      if (nextLocation[3] == position::down) {//else {
        stepperE0.setTarget(pos_down);
        nowLocation[3] = position::unknown;
        //homing(axis::X);
        //Serial.println("targetE0 down");

        //
//println("..................doing to down");
      }
  }else{
      //Serial.println(stepperX.pos);
     // AllJobsDone = true;
     if(power_OFF_while_STANDING){
      stepperE0.disable();
     }
     mtrToMove[3]=true;
    }
  }else{
    mtrToMove[3]=false;
    //AllJobsDone = false;
  }

 

  if (stepperE1.getStatus() == 0) {
    //Serial.println("status = 0");
    if (stepperE1.pos == pos_top*2) {
      nowLocation[4] = position::top;
    }
    if (stepperE1.pos == pos_down) {
      nowLocation[4] = position::down;
    }
    if (nowLocation[4] != nextLocation[4] && !toRESET) {
      if(power_OFF_while_STANDING){
        stepperE1.enable();  
      }
      if (nextLocation[4] == position::top) {
        stepperE1.setTarget(pos_top*2);
        nowLocation[4] = position::unknown;
        //Serial.println("..................doing to top");
      }
      if (nextLocation[4] == position::down) {//else {
        stepperE1.setTarget(pos_down);
        nowLocation[4] = position::unknown;
        //homing(axis::X);
        //Serial.println("targetE1 down");

        //Serial.println("..................doing to down");
      }
  }else{
      //Serial.println(stepperX.pos);
     // AllJobsDone = true;
     if(power_OFF_while_STANDING){
      stepperE1.disable();
     }
     mtrToMove[4]=true;
    }
  }else{
    mtrToMove[4]=false;
    //AllJobsDone = false;
  }

  
  /* if (nowLocation[0] == position::down && nextLocation[0] == position::top) {
     nowLocation[0] = position::top;
    }*/



  stepperX.tick();   // мотор асинхронно крутится тут
  stepperY.tick();
  stepperZ.tick();
  stepperE0.tick();
  stepperE1.tick();




  // асинхронный вывод в порт
  
  if (millis() - tmr >= randomizing_delay) {
    if (AllJobsDone) {
      //Serial.println("AllJobsDone");
    }
    else {
      //Serial.println("NOT AllJobsDone");
    }
    tmr = millis();

    //if (AllJobsDone == true) { /////// / // / / / // // // / ALL TIME RANDOMIZING 
      //Serial.println("Rndoming");
      //rndMotorsToMoove=random(0,5);
      for (int i = 0; i < 5; i++) {
        if(mtrToMove[i]){
        rndMotorsToMoove = random(4);
        //Serial.println(rndMotorsToMoove);
        if (rndMotorsToMoove == 1) {
          //Serial.println("GO");
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

void softReset() {
  digitalWrite(AUX2_10_PIN,LOW);
}
