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

#include <Wire.h>                                     // подключаем библиотеку для работы с шиной I2C

#define DRIVER_STEP_TIME 10  // меняем задержку на 10 мкс

#define GS_FAST_PROFILE 10
#include "GyverStepper2.h"

uint32_t tar = 80000;
bool dir = 1;

#define REQUIRE_MEGA2560
#include "macros.h"
#include "pins_MKS_GEN_13.h"


#define MAX_SPEED 10000
#define ACCEL 10000

bool dirX, dirY, dirZ, dirE0, dirE1;

enum class axis {X, Y, Z, E0, E1, ALL};

//enum class state {mooving, stay};

enum class position {unknown, down, top};

position nowLocation[5] = {position::unknown, position::unknown, position::unknown, position::unknown, position::unknown};
position nextLocation[5] = {position::top, position::top, position::top, position::top, position::top};
//state motorsStatus[5] = {state::stay, state::stay, state::stay, state::stay, state::stay};

bool wireSet[5] = {false, false, false, false, false};
bool wireDone[5] = {true, true, true, true, true};


GStepper2 <STEPPER2WIRE> stepperX(2048, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperY(2048, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperZ(2048, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperE0(2048, E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN);
GStepper2 <STEPPER2WIRE> stepperE1(2048, E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN);



void setup() {
  pinMode(X_MIN_PIN, INPUT_PULLUP);
  pinMode(X_MAX_PIN, INPUT);
  pinMode(Y_MIN_PIN, INPUT);
  pinMode(Y_MAX_PIN, INPUT);
  pinMode(Z_MIN_PIN, INPUT);
  pinMode(Z_MAX_PIN, INPUT);

  //pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_CS_PIN, OUTPUT);
  pinMode(Y_CS_PIN, OUTPUT);
  pinMode(Z_CS_PIN, OUTPUT);
  pinMode(E0_CS_PIN, OUTPUT);
  pinMode(E1_CS_PIN, OUTPUT);
  //pinMode(Y_ENABLE_PIN, OUTPUT);

  //MASTER
  Wire.setClock(400000);                              // устанавливаем скорость передачи данных по шине I2C = 400кБит/с
  Wire.begin();                                       // инициируем подключение к шине I2C в качестве ведущего (master) устройства.

  Serial.begin(9600);

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
  stepperX.setTarget(-tar);
  stepperY.setTarget(-tar);
  stepperZ.setTarget(-tar);
  stepperE0.setTarget(-tar);
  stepperE1.setTarget(-tar);
}

int x = 0;
void loop() {
  //MASTER
  wave();
  Serial.println("I2c sended...");
  delay(2000);


if (stepperX.getStatus()==0) {
    if (nowLocation[0] != nextLocation[0]) {
      if (nextLocation[0] == position::top) {
        stepperX.setTarget(-tar);
        Serial.println("targetX top");
        wireDone[0]=false;
      }
      else {
        wireDone[0]=false;
        homing(axis::X);
        nextLocation[0] = position::down;
        Serial.println("targetX down");
      }
    }
    if (nowLocation[0] == position::down && nextLocation[0] == position::top) {
      nowLocation[0] = position::top;
    }
  }
  //Y
  if (stepperY.ready()) {
    if (nowLocation[1] != nextLocation[1]) {
      if (nextLocation[1] == position::top) {
        stepperY.setTarget(-tar);
        wireDone[1]=false;
      }
      else {
        wireDone[1]=false;
        homing(axis::Y);
        nextLocation[1] = position::down;
      }
    }
    if (nowLocation[1] == position::down && nextLocation[1] == position::top) {
      nowLocation[1] = position::top;
    }
  }
  //Z
  if (stepperZ.ready()) {
    if (nowLocation[2] != nextLocation[2]) {
      if (nextLocation[2] == position::top) {
        wireDone[2]=false;
        stepperZ.setTarget(-tar);
      }
      else {
        wireDone[2]=false;
        homing(axis::Z);
        nextLocation[2] = position::down;
      }
    }
    if (nowLocation[2] == position::down && nextLocation[2] == position::top) {
      nowLocation[2] = position::top;
    }
  }
  //E0
  if (stepperE0.ready()) {
    if (nowLocation[3] != nextLocation[3]) {
      if (nextLocation[3] == position::top) {
        wireDone[3]=false;
        stepperE0.setTarget(-tar);
      }
      else {
        wireDone[3]=false;
        homing(axis::E0);
        nextLocation[3] = position::down;
      }
    }
    if (nowLocation[3] == position::down && nextLocation[3] == position::top) {
      nowLocation[3] = position::top;
    }
  }
  //E1
  if (stepperE1.ready()) {
    if (nowLocation[4] != nextLocation[4]) {
      if (nextLocation[4] == position::top) {
        wireDone[4]=false;
        stepperE1.setTarget(-tar);
      }
      else {
        wireDone[4]=false;
        homing(axis::E1);
        nextLocation[4] = position::down;
      }
    }
    if (nowLocation[4] == position::down && nextLocation[4] == position::top) {
      nowLocation[4] = position::top;
    }
  }

/*
  
  if(stepperX.ready()){
    if (nowLocation[0] == position::down && nextLocation[0]==position::top){
      nowLocation[0]=position::top;
    }
  }
  if(stepperY.ready()){
    if (nowLocation[1] == position::down && nextLocation[1]==position::top){
      nowLocation[1]=position::top;
    }
  }
  if(stepperZ.ready()){
    if (nowLocation[2] == position::down && nextLocation[2]==position::top){
      nowLocation[2]=position::top;
    }
  }
  if(stepperE0.ready()){
    if (nowLocation[3] == position::down && nextLocation[3]==position::top){
      nowLocation[3]=position::top;
    }
  }
  if(stepperE1.ready()){
    if (nowLocation[4] == position::down && nextLocation[4]==position::top){
      nowLocation[4]=position::top;
    }
  }

  //X
  if (nowLocation[0] != nextLocation[0] && motorsStatus) {
    if (nextLocation[0] == position::top) {
      stepperX.setTarget(-tar);
    }
    else {
      stepperX.setTarget(tar);
      nextLocation[0] = position::down;
    }
  }
  //Y
  if (nowLocation[1] != nextLocation[1]) {
    if (nextLocation[1] == position::top) {
      stepperY.setTarget(-tar);
    }
    else {
      stepperY.setTarget(tar);
      nextLocation[1] = position::down;
    }
  }
  //Z
  if (nowLocation[2] != nextLocation[2]) {
    if (nextLocation[2] == position::top) {
      stepperZ.setTarget(-tar);
    }
    else {
      stepperZ.setTarget(tar);
      nextLocation[2] = position::down;
    }
  }
  //E0
  if (nowLocation[3] != nextLocation[3]) {
    if (nextLocation[3] == position::top) {
      stepperE0.setTarget(-tar);
    }
    else {
      stepperE0.setTarget(tar);
      nextLocation[3] = position::down;
    }
  }
  //E1
  if (nowLocation[4] != nextLocation[4]) {
    if (nextLocation[4] == position::top) {
      stepperE1.setTarget(-tar);
    }
    else {
      stepperE1.setTarget(tar);
      nextLocation[4] = position::down;
    }
  }
*/

  stepperX.tick();   // мотор асинхронно крутится тут
  stepperY.tick();
  stepperZ.tick();
  stepperE0.tick();
  stepperE1.tick();

  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= 300) {
    tmr = millis();
    Serial.println(stepperX.pos);
  }

  

}

void wave() {
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);                // position {unknown, down, top};  0 1 2
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

  delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);                // position {unknown, down, top};  0 1 2
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

  delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);                // position {unknown, down, top};  0 1 2
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

  delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);                // position {unknown, down, top};  0 1 2
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

  delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting
delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.write(2);
  Wire.endTransmission();    // stop transmitting

delay(2000);

//////////////////////////////////////////////////////
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();    // stop transmitting

  delay(1000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();    // stop transmitting

delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();    // stop transmitting
delay(2000);
  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();    // stop transmitting
delay(2000);

  Wire.beginTransmission(0x01); // transmit to device #9
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.write(1);
  Wire.endTransmission();    // stop transmitting

delay(2000);


}

void homing(axis homer) {
  if (homer == axis::X || homer == axis::ALL) {
    Serial.println("Homing.....");
    Serial.println("Homing X");
    
    if (digitalRead(X_MIN_PIN)) {       // если концевик X не нажат
      //stepperX.setSpeed(-2000);       // ось Х, -10 шаг/сек
      stepperX.setTarget(-80000);
      while (digitalRead(X_MIN_PIN)) {  // пока кнопка не нажата
        stepperX.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperX.brake();                // тормозим, приехали
    }
    stepperX.setCurrent(0); 
    nowLocation[0] = position::down;
    Serial.println("X Home!");
  }
  if (homer == axis::Y || homer == axis::ALL) {
    Serial.println("Homing Y");
    stepperY.reset();    // сбрасываем координаты в 0
    if (digitalRead(X_MAX_PIN)) {       // если концевик X не нажат
      stepperY.setSpeed(-10);       // ось Х, -10 шаг/сек
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
    stepperZ.reset();    // сбрасываем координаты в 0
    if (digitalRead(Y_MIN_PIN)) {       // если концевик X не нажат
      stepperZ.setSpeed(-10);       // ось Х, -10 шаг/сек
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
      stepperE0.setSpeed(-10);       // ось Х, -10 шаг/сек
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
      stepperE1.setSpeed(-10);       // ось Х, -10 шаг/сек
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
}
