// пример калибровки нуля по концевикам
// концевик на D6

#include "macros.h"
#include "pins_BTT_OCTOPUS_PRO_V1_1.h"


#define LIMSW_X 6

#include "GyverStepper2.h"
//GStepper2<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper2 <STEPPER2WIRE> stepper(2048, X_STEP_PIN, X_DIR_PIN);

void setup() {
  // пуллапим. Кнопки замыкают на GND
   pinMode(X_DIAG_PIN , INPUT_PULLUP);
  pinMode(Y_DIAG_PIN, INPUT_PULLUP);

  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(X_CS_PIN, OUTPUT);


  pinMode(Y_ENABLE_PIN, OUTPUT);
  pinMode(Y_CS_PIN, OUTPUT);

}

void loop() {
}

void homing() {
  if (digitalRead(X_DIAG_PIN )) {       // если концевик X не нажат
    stepper.setSpeed(-10.);       // ось Х, -10 шаг/сек
    while (digitalRead(X_DIAG_PIN)) {  // пока кнопка не нажата
      stepper.tick();               // крутим
    }
    // кнопка нажалась - покидаем цикл
    stepper.brake();                // тормозим, приехали
  }
  stepper.reset();    // сбрасываем координаты в 0
}
