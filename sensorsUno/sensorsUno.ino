#include <QTRSensors.h>

// Sensor array (RC-type sensors)
QTRSensorsRC qtr((const uint8_t[]){A0, A1, A2, A3, A4, A5, 5, 6}, 8);  // Pass sensor pins directly to the constructor
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup(){
  Serial.begin(9600);  // Initialize serial communication first

  // Calibrate sensors
  calibrateSensors();
}

void loop(){
  // Read sensor values and get the position of the white line
  uint16_t position = qtr.readLineWhite(sensorValues);

  // Print sensor values
  for (int i = 0; i < SensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();

  // Print the calculated line position
  Serial.print("Line position: ");
  Serial.println(position);

  delay(100);  // Small delay to avoid flooding the serial output
}

void calibrateSensors(){
  digitalWrite(LED_BUILIIN, HIGH);
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); //calibration takes 10s, led will be on for 10s
}
