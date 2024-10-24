#include <QTRSensors.h>

// Polulu sensor setup
QTRSensors qtrrc;
#define sensorCount 12
uint16_t sensorValues[sensorCount];  // Array to store sensor values

void setup(){
  // put your setup code here, to run once:
  // Initialize sensor array
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){ 13, 15, 12, 4, 27, 16, 26, 17, 25, 5, 33, 18 }, 12);
  Serial.begin(9600);

  for (int i = 0; i < 250; i++) {
    qtrrc.calibrate();
    delay(20);
    Serial.println("Calibrating...");
  }
}

void loop(){
  // put your main code here, to run repeatedly:
  qtrrc.readLineWhite(sensorValues);
  for (int i = 0; i < sensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}
