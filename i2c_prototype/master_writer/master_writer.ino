#include <Wire.h>
#define i2c_unknown 0
#define i2c_set 1
#define i2c_gotit 2
#define i2c_doing 3
#define i2c_done 4

#define PAYLOAD_SIZE 10
byte Dev[6][5] = {
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
};
byte DevStat[6][5] = {
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
};
byte DevSend[6][5] = {
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
  {0, 0, 0, 0, 0},
};




bool sendsTO[6] = {false, false, false, false, false, false};
byte DevAddrses[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
static uint32_t tmr;

void setup() {
  //Wire.setClock(400000);                              // устанавливаем скорость передачи данных по шине I2C = 400кБит/с
  Wire.begin();                                       // инициируем подключение к шине I2C в качестве ведущего (master) устройства.
  Serial.begin(9600);
  Serial.println("Master begin");

  tmr = millis();
}


int DevStairsCounter=0;

void stairs(){
  for (int i = 0; i <= 4; i++) {
    if (Dev[i][DevStairsCounter] == 1){
       DevSend[i][DevStairsCounter]=2;
    }
    if (Dev[i][DevStairsCounter] == 2){
       DevSend[i][DevStairsCounter]=1;
    }
  }
}


void loop() {

  if (millis() - tmr >= 1000) {
    for (int i = 0; i <= 4; i++) {
      Wire.requestFrom(DevAddrses[i], PAYLOAD_SIZE);    // request 6 bytes from slave device #8
      if (Wire.available() == PAYLOAD_SIZE) { // slave may send less than requested
        Dev[i][0] = Wire.read();
        Dev[i][1] = Wire.read();
        Dev[i][2] = Wire.read();
        Dev[i][3] = Wire.read();
        Dev[i][4] = Wire.read();

        DevStat[i][0] = Wire.read();
        DevStat[i][1] = Wire.read();
        DevStat[i][2] = Wire.read();
        DevStat[i][3] = Wire.read();
        DevStat[i][4] = Wire.read();

        Serial.print(i);
        Serial.print(" i2c get: ");
        Serial.print(Dev[i][0]);
        Serial.print(" ");
        Serial.print(Dev[i][1]);
        Serial.print(" ");
        Serial.print(Dev[i][2]);
        Serial.print(" ");
        Serial.print(Dev[i][3]);
        Serial.print(" ");
        Serial.print(Dev[i][4]);
        Serial.print(" ..." );
        Serial.print(DevStat[i][0]);
        Serial.print(" ");
        Serial.print(DevStat[i][1]);
        Serial.print(" ");
        Serial.print(DevStat[i][2]);
        Serial.print(" ");
        Serial.print(DevStat[i][3]);
        Serial.print(" ");
        Serial.print(DevStat[i][4]);
        Serial.println(" ..." );



        for (int j = 0; j <= 4; j++) {
          if (DevStat[i][j] == i2c_unknown || DevStat[i][j] == i2c_done) {
            if (Dev[i][j] == 0 ) {
              sendsTO[i] = true;
              DevSend[i][j] = 2;
              DevStat[i][j] = i2c_set;
              
            }
            if (Dev[i][j] == 1 && random(0,2)==1) {
              sendsTO[i] = true;
              DevSend[i][j] = 2;
              DevStat[i][j] = i2c_set;

            }
            if (Dev[i][j] == 2&& random(0,2)==1) {
              sendsTO[j] = true;
              DevSend[i][j] = 1;
              DevStat[i][j] = i2c_set;

            }
          }

        }

      }
    }

    //Wire.endTransmission();
    tmr = millis();

  }
  delay(1000);
  for (int i = 0; i <= 4; i++) {
    if (sendsTO[i]) {
      Serial.print(i);
      Serial.print("i2c sending to ");

  Serial.print(DevSend[i][0]);
        Serial.print(" ");
        Serial.print(DevSend[i][1]);
        Serial.print(" ");
        Serial.print(DevSend[i][2]);
        Serial.print(" ");
        Serial.print(DevSend[i][3]);
        Serial.print(" ");
        Serial.print(DevSend[i][4]);
        Serial.print(" ..." );
        Serial.print(DevStat[i][0]);
        Serial.print(" ");
        Serial.print(DevStat[i][1]);
        Serial.print(" ");
        Serial.print(DevStat[i][2]);
        Serial.print(" ");
        Serial.print(DevStat[i][3]);
        Serial.print(" ");
        Serial.print(DevStat[i][4]);
        Serial.println(" ..." );

      
      Wire.beginTransmission(DevAddrses[i]);
      Wire.write(DevSend[i][0]);                // position {unknown, down, top};  0 1 2
      Wire.write(DevSend[i][1]);
      Wire.write(DevSend[i][2]);
      Wire.write(DevSend[i][3]);
      Wire.write(DevSend[i][4]);

      Wire.write(DevStat[i][0]);                // position {unknown, down, top};  0 1 2
      Wire.write(DevStat[i][1]);
      Wire.write(DevStat[i][2]);
      Wire.write(DevStat[i][3]);
      Wire.write(DevStat[i][4]);
      Wire.endTransmission();    // stop transmitting

      sendsTO[i] = false;
    }
  }

}
