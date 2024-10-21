#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 12;
uint16_t sensorValues[SensorCount];
int led_pin = 10;

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){}, SensorCount);
  pinMode(led_pin, OUTPUT);
  calibrate_sensor();
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  int position = qtr.readLineWhite(sensorValues);

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println(position);
}

void calibrate_sensor(){
  digitalWrite(led_pin, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led_pin, LOW);
}
