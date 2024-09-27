#include <QTRSensors.h>

// Sensor array
QTRSensors qtr;  // Generic QTRSensors object
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){
  Serial.begin(9600);  // Start serial communication

  // Configure sensor array for RC type sensors
  qtr.setTypeRC();  // Set sensor type to RC
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, 5, 6}, SensorCount);  // Set sensor pins
  
  // Calibrate the sensors
  calibrateSensors();
}

void loop(){
  // Read sensor values and get the position of the white line
  uint16_t position = qtr.readLineBlack(sensorValues);

  // Print sensor values
  for (int i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();


  delay(100);  // Small delay to avoid flooding the serial output
}


void calibrateSensors(){
  digitalWrite(LED_BUILTIN, HIGH);
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //calibration takes 10s, led will be on for 10s
}
