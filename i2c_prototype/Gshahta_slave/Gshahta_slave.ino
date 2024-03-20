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
#include <Wire.h>
#define I2C_SLAVE_ADDRESS 0x01
byte REG_Array[5] = {0, 0, 0, 0, 0};

byte REG_Array_stat[5] = {0, 0, 0, 0, 0};  //   0 xz 1 set 2 get 3 done
#define i2c_unknown 0
#define i2c_set 1
#define i2c_gotit 2
#define i2c_doing 3
#define i2c_done 4

//{0, 0, 0, 0, 0,0,0,0,0,0};
#define PAYLOAD_SIZE 10
////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////// drivers

#define DRIVER_STEP_TIME 10  // меняем задержку на 10 мкс

#define GS_FAST_PROFILE 10
#include "GyverStepper2.h"

#define pos_top 60000
#define pos_down 0

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
position nextLocation[5] = {position::unknown, position::unknown, position::unknown, position::unknown, position::unknown};

//position setLocation[5] =  {position::unknown, position::unknown, position::unknown, position::unknown, position::unknown};

//state motorsStatus[5] = {state::stay, state::stay, state::stay, state::stay, state::stay};

bool wireSet[5] = {false, false, false, false, false};
bool wireDone[5] = {true, true, true, true, true};

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

  homing(axis::ALL);
  /*stepperX.setTarget(-tar);
    stepperY.setTarget(-tar);
    stepperZ.setTarget(-tar);
    stepperE0.setTarget(-tar);
    stepperE1.setTarget(-tar);*/
  //Slave
  //  Wire.setClock(400000);                              // устанавливаем скорость передачи данных по шине I2C = 400кБит/с
  Wire.begin(I2C_SLAVE_ADDRESS);                                   // инициируем подключение к шине I2C в качестве ведомого (slave) устройства, с указанием своего адреса на шине.
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent); // register event
}

void receiveEvent(int bytes) {
  if (Wire.available() == PAYLOAD_SIZE) {
    REG_Array[0] = Wire.read();    // read one character from the I2C
    REG_Array[1] = Wire.read();
    REG_Array[2] = Wire.read();
    REG_Array[3] = Wire.read();
    REG_Array[4] = Wire.read();

    REG_Array_stat[0] = Wire.read();
    REG_Array_stat[1] = Wire.read();
    REG_Array_stat[2] = Wire.read();
    REG_Array_stat[3] = Wire.read();
    REG_Array_stat[4] = Wire.read();

    Serial.print("i2c get: ");


    if (REG_Array[0] < 0 || REG_Array[0] > 2) {
      Serial.println("bad value ");
    }
    if (REG_Array[1] < 0 || REG_Array[1] > 2) {
      Serial.println("bad value ");
    }
    if (REG_Array[2] < 0 || REG_Array[2] > 2) {
      Serial.println("bad value ");
    }
    if (REG_Array[3] < 0 || REG_Array[3] > 2) {
      Serial.println("bad value ");
    }
    if (REG_Array[4] < 0 || REG_Array[4] > 2) {
      Serial.println("bad value ");
    }

    if (REG_Array_stat[0] < i2c_set || REG_Array_stat[0] > i2c_done) {
      Serial.println("bad stat value ");
    }
    if (REG_Array_stat[1] < i2c_set || REG_Array_stat[1] > i2c_done) {
      Serial.println("bad stat value ");
    }
    if (REG_Array_stat[2] < i2c_set || REG_Array_stat[2] > i2c_done) {
      Serial.println("bad stat value ");
    }
    if (REG_Array_stat[3] < i2c_set || REG_Array_stat[3] > i2c_done) {
      Serial.println("bad stat value ");
    }
    if (REG_Array_stat[4] < i2c_set || REG_Array_stat[4] > i2c_done) {
      Serial.println("bad stat value ");
    }
    Serial.print(REG_Array[0]); Serial.print(" ");
    Serial.print(REG_Array[1]); Serial.print(" ");
    Serial.print(REG_Array[2]); Serial.print(" ");
    Serial.print(REG_Array[3]); Serial.print(" ");
    Serial.print(REG_Array[4]); Serial.println(" ..." );

    if (wireDone[0] && REG_Array_stat[0] == i2c_set) {
      nextLocation[0] = static_cast<position>(REG_Array[0]);
      REG_Array_stat[0] = i2c_gotit;
      //      Serial.print((int)nextLocation[0));
    }
    if (wireDone[1] && REG_Array_stat[1] == i2c_set) {
      nextLocation[1] = static_cast<position>(REG_Array[1]);
      REG_Array_stat[1] = i2c_gotit;

    }
    if (wireDone[2] && REG_Array_stat[2] == i2c_set) {
      nextLocation[2] = static_cast<position>(REG_Array[2]);
      REG_Array_stat[2] = i2c_gotit;

    }
    if (wireDone[3] && REG_Array_stat[3] == i2c_set) {
      nextLocation[3] = static_cast<position>(REG_Array[3]);
      REG_Array_stat[3] = i2c_gotit;

    }
    if (wireDone[4] && REG_Array_stat[4] == i2c_set) {
      nextLocation[4] = static_cast<position>(REG_Array[4]);
      REG_Array_stat[4] = i2c_gotit;
    }

  }
}

void requestEvent() {
  Serial.println("wire send...");
  Wire.write(REG_Array[0]);
  Wire.write(REG_Array[1]);
  Wire.write(REG_Array[2]);
  Wire.write(REG_Array[2]);
  Wire.write(REG_Array[3]);

  Wire.write(REG_Array_stat[0]);
  Wire.write(REG_Array_stat[1]);
  Wire.write(REG_Array_stat[2]);
  Wire.write(REG_Array_stat[3]);
  Wire.write(REG_Array_stat[4]);
}

void loop() {
  //X
  /*
  */

  if (stepperX.getStatus() == 0) {
    if (stepperX.pos == pos_top) {
      nowLocation[0] = position::top;
      if (REG_Array_stat[0] == i2c_doing) {
        REG_Array_stat[0] = i2c_done;
        REG_Array[0] = (int)position::top;
        wireDone[0] = true;
        //Serial.println("---------/ontop----");
      }
    }
    if (stepperX.pos == pos_down) {
      nowLocation[0] = position::down;
      if (REG_Array_stat[0] == i2c_doing) {
        REG_Array_stat[0] = i2c_done;
        REG_Array[0] = (int)position::down;
        wireDone[0] = true;
        //Serial.println("---------/ondown----");
      }
    }
    if (REG_Array_stat[0] == i2c_gotit) {
      //Serial.print(" ///////////////////////gotit----");
      //Serial.println((int)nextLocation[0]);
      if (nowLocation[0] != nextLocation[0]) {
        if (nextLocation[0] == position::top) {
          stepperX.setTarget(pos_top);
          nowLocation[0] = position::unknown;
          //Serial.println("targetX top");
          REG_Array_stat[0] = i2c_doing;
          wireDone[0] = false;
          //Serial.println("..................doing to top");
        }
        if (nextLocation[0] == position::down) {//else {
          stepperX.setTarget(pos_down);
          nowLocation[0] = position::unknown;
          //homing(axis::X);
          //Serial.println("targetX down");
          REG_Array_stat[0] = i2c_doing;
          wireDone[0] = false;
          //Serial.println("..................doing to down");
        }
      }
    }
  }


  if (stepperY.getStatus() == 0) {
    if (stepperY.pos == pos_top) {
      nowLocation[1] = position::top;
      if (REG_Array_stat[1] == i2c_doing) {
        REG_Array_stat[1] = i2c_done;
        REG_Array[1] = (int)position::top;
        wireDone[1] = true;
        //Serial.println("---------/ontop----");
      }
    }
    if (stepperY.pos == pos_down) {
      nowLocation[1] = position::down;
      if (REG_Array_stat[1] == i2c_doing) {
        REG_Array_stat[1] = i2c_done;
        REG_Array[1] = (int)position::down;
        wireDone[1] = true;
        //Serial.println("---------/ondown----");
      }
    }
    if (REG_Array_stat[1] == i2c_gotit) {
      //Serial.print(" ///////////////////////gotit----");
      //Serial.println((int)nextLocation[1]);
      if (nowLocation[1] != nextLocation[1]) {
        if (nextLocation[1] == position::top) {
          stepperY.setTarget(pos_top);
          nowLocation[1] = position::unknown;
          //Serial.println("targetX top");
          REG_Array_stat[1] = i2c_doing;
          wireDone[1] = false;
          //Serial.println("..................doing to top");
        }
        if (nextLocation[1] == position::down) {//else {
          stepperY.setTarget(pos_down);
          nowLocation[1] = position::unknown;
          //homing(axis::X);
          //Serial.println("targetX down");
          REG_Array_stat[1] = i2c_doing;
          wireDone[1] = false;
          //Serial.println("..................doing to down");
        }
      }
    }
  }

  
  if (stepperZ.getStatus() == 0) {
    if (stepperZ.pos == pos_top) {
      nowLocation[2] = position::top;
      if (REG_Array_stat[2] == i2c_doing) {
        REG_Array_stat[2] = i2c_done;
        REG_Array[2] = (int)position::top;
        wireDone[2] = true;
        //Serial.println("---------/ontop----");
      }
    }
    if (stepperZ.pos == pos_down) {
      nowLocation[2] = position::down;
      if (REG_Array_stat[2] == i2c_doing) {
        REG_Array_stat[2] = i2c_done;
        REG_Array[2] = (int)position::down;
        wireDone[2] = true;
        //Serial.println("---------/ondown----");
      }
    }
    if (REG_Array_stat[2] == i2c_gotit) {
      //Serial.print(" ///////////////////////gotit----");
      //Serial.println((int)nextLocation[2]);
      if (nowLocation[2] != nextLocation[2]) {
        if (nextLocation[2] == position::top) {
          stepperZ.setTarget(pos_top);
          nowLocation[2] = position::unknown;
          //Serial.println("targetX top");
          REG_Array_stat[2] = i2c_doing;
          wireDone[2] = false;
          //Serial.println("..................doing to top");
        }
        if (nextLocation[2] == position::down) {//else {
          stepperZ.setTarget(pos_down);
          nowLocation[2] = position::unknown;
          //homing(axis::X);
          //Serial.println("targetX down");
          REG_Array_stat[2] = i2c_doing;
          wireDone[2] = false;
          //Serial.println("..................doing to down");
        }
      }
    }
  }
  if (stepperE0.getStatus() == 0) {
    if (stepperE0.pos == pos_top) {
      nowLocation[3] = position::top;
      if (REG_Array_stat[3] == i2c_doing) {
        REG_Array_stat[3] = i2c_done;
        REG_Array[3] = (int)position::top;
        wireDone[3] = true;
        //Serial.println("---------/ontop----");
      }
    }
    if (stepperE0.pos == pos_down) {
      nowLocation[3] = position::down;
      if (REG_Array_stat[3] == i2c_doing) {
        REG_Array_stat[3] = i2c_done;
        REG_Array[3] = (int)position::down;
        wireDone[3] = true;
        //Serial.println("---------/ondown----");
      }
    }
    if (REG_Array_stat[3] == i2c_gotit) {
      //Serial.print(" ///////////////////////gotit----");
      //Serial.println((int)nextLocation[3]);
      if (nowLocation[3] != nextLocation[3]) {
        if (nextLocation[3] == position::top) {
          stepperE0.setTarget(pos_top);
          nowLocation[3] = position::unknown;
          //Serial.println("targetX top");
          REG_Array_stat[3] = i2c_doing;
          wireDone[3] = false;
          //Serial.println("..................doing to top");
        }
        if (nextLocation[3] == position::down) {//else {
          stepperE0.setTarget(pos_down);
          nowLocation[3] = position::unknown;
          //homing(axis::X);
          //Serial.println("targetX down");
          REG_Array_stat[3] = i2c_doing;
          wireDone[3] = false;
          //Serial.println("..................doing to down");
        }
      }
    }
  }
  if (stepperE1.getStatus() == 0) {
    if (stepperE1.pos == pos_top) {
      nowLocation[4] = position::top;
      if (REG_Array_stat[4] == i2c_doing) {
        REG_Array_stat[4] = i2c_done;
        REG_Array[4] = (int)position::top;
        wireDone[4] = true;
        //Serial.println("---------/ontop----");
      }
    }
    if (stepperE1.pos == pos_down) {
      nowLocation[4] = position::down;
      if (REG_Array_stat[4] == i2c_doing) {
        REG_Array_stat[4] = i2c_done;
        REG_Array[4] = (int)position::down;
        wireDone[4] = true;
        //Serial.println("---------/ondown----");
      }
    }
    if (REG_Array_stat[4] == i2c_gotit) {
      //Serial.print(" ///////////////////////gotit----");
      //Serial.println((int)nextLocation[4]);
      if (nowLocation[4] != nextLocation[4]) {
        if (nextLocation[4] == position::top) {
          stepperE1.setTarget(pos_top);
          nowLocation[4] = position::unknown;
          //Serial.println("targetX top");
          REG_Array_stat[4] = i2c_doing;
          wireDone[4] = false;
          //Serial.println("..................doing to top");
        }
        if (nextLocation[4] == position::down) {//else {
          stepperE1.setTarget(pos_down);
          nowLocation[4] = position::unknown;
          //homing(axis::X);
          //Serial.println("targetX down");
          REG_Array_stat[4] = i2c_doing;
          wireDone[4] = false;
          //Serial.println("..................doing to down");
        }
      }
    }
    /* if (nowLocation[0] == position::down && nextLocation[0] == position::top) {
       nowLocation[0] = position::top;
      }*/
  }


  stepperX.tick();   // мотор асинхронно крутится тут
  stepperY.tick();
  stepperZ.tick();
  stepperE0.tick();
  stepperE1.tick();



  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= 900) {
    tmr = millis();
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
    }
  }

}

void homing(axis homer) {
  Serial.println("Homing.....");
  if (homer == axis::X || homer == axis::ALL) {
    Serial.println("Homing X");

    if (digitalRead(X_MIN_PIN)) {       // если концевик X не нажат
      stepperX.setSpeed(-20000);       // ось Х, -10 шаг/сек
      //stepperX.setTarget(800000);
      while (digitalRead(X_MIN_PIN)) {  // пока кнопка не нажата
        stepperX.tick();               // крутим
      }
      // кнопка нажалась - покидаем цикл
      stepperX.brake();                // тормозим, приехали
    }
    stepperX.reset();    // сбрасываем координаты в 0
    nowLocation[0] = position::down;
    REG_Array[0] = (int)position::down;
    Serial.println("X Home!");
  }
  if (homer == axis::Y || homer == axis::ALL) {
    Serial.println("Homing Y");
    stepperY.reset();    // сбрасываем координаты в 0
    if (digitalRead(X_MAX_PIN)) {       // если концевик X не нажат
      stepperY.setSpeed(-20000);       // ось Х, -10 шаг/сек
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
    stepperZ.setTarget(tar);
    stepperZ.reset();    // сбрасываем координаты в 0
    if (digitalRead(Y_MIN_PIN)) {       // если концевик X не нажат
      stepperZ.setSpeed(-20000);       // ось Х, -10 шаг/сек
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
      stepperE0.setSpeed(-20000);       // ось Х, -10 шаг/сек
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
      stepperE1.setSpeed(-20000);       // ось Х, -10 шаг/сек
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
