// imports
#include <BluetoothSerial.h>
#include <QTRSensors.h>

// ESP - Motor Pin Connections
#define LH 19 // IN1
#define LL 23 // IN2
#define RH 21 // IN3
#define RL 2 // IN4


// Pololu sensor setup
#define sensorCount 12
QTRSensors qtrrc;
int basespeed = 75; // Base motor speed
int maxSpeed = 100; // Maximum motor speed
uint16_t sensorValues[sensorCount]; // Array to store sensor values

// PID constants
float Kp = 0.005;  // Proportional gain
float Ki = 0.0;  // Integral gain
float Kd = 0.05;  // Derivative gain
int integral = 0;
int lastError = 0;
char myData[30] = { 0 } , s1[10], s2[10],s3[10],s4[10];

void setup() {
  // Set motor pins as outputs
  pinMode(RH, OUTPUT);
  pinMode(RL, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(LH, OUTPUT);

  // Initialize sensor array
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){13, 15, 14, 4, 27, 16, 26, 17, 25, 5, 33, 18}, sensorCount);
  Serial.begin(9600);
 // Serial.begin("PID_Tuner"); // Bluetooth device name

  Serial.println("Bluetooth Device is Ready to Pair");
 // Serial.println("Bluetooth Device is Ready to Pair");

  // Calibrate sensors
  for (int i = 0; i < 250; i++) {
    qtrrc.calibrate();
    delay(20);
    Serial.println("Calibrating...");
  }
}

// int counter = 0;
void loop() {
  Serial.println(""); 
  // counter += 1;

  // read BT data and check it
  if (Serial.available()){
    byte m = Serial.readBytesUntil('\n', myData, 20);
    myData[m] = '\0';  //null-byte
    if (sscanf(myData, "%[^','],%[^','],%[^','],%[^','],%s", s1, s2,s4,s3) == 4) {
      Kp = atof(s1);
      Kd = atof(s2);
      maxSpeed = atof(s3);
      basespeed=atof(s4);
      Serial.println("values updated");
    }
    else
      Serial.println("error in input!");
  }
  

  // Read sensor values
  int position = qtrrc.readLineWhite(sensorValues);  // Get position based on weighted sum of sensor readings


  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
  integral += error;  // Calculate integral component
  int derivative = error - lastError;  // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;  // Store current error as lastError for next iteration

  // Adjust motor speeds based on PID value
  int leftMotorSpeed = basespeed + turn;
  int rightMotorSpeed = basespeed - turn;

  // Ensure motor speeds are within bounds
  if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
  if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
  // if (leftMotorSpeed < 0 && leftMotorSpeed > -70) leftMotorSpeed = 0;
  // if (rightMotorSpeed < 0  && rightMotorSpeed > -70) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = -basespeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = -basespeed;  

  // Motor control logic
  if (leftMotorSpeed < 0) {
    analogWrite(LL, leftMotorSpeed);  // Reverse left motor
    analogWrite(LH, 0);
  } else {
    analogWrite(LH, leftMotorSpeed);
    analogWrite(LL, 0);
  }

  if (rightMotorSpeed < 0) {
    analogWrite(RL, rightMotorSpeed);  // Reverse right motor
    analogWrite(RH, 0);
  } else {
    analogWrite(RH, rightMotorSpeed);
    analogWrite(RL, 0);
  }


// if (counter % 10 == 0) {
  for (int i = 0; i < 12; i++) {
    Serial.print(sensorValues[i] > 500 ? 1 : 0);
    // Serial.print(" ");
  }
  Serial.print(" ");
// }

  // Output debugging information
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print("\t");
  // Serial.print("Left Speed: ");
  // Serial.print(leftMotorSpeed);
  // Serial.print("\t");
  // Serial.print("Right Speed: ");
  // Serial.print(rightMotorSpeed);
  // Serial.print("\t");
  // Serial.print(Kp);
  // Serial.print("\t");
  // Serial.print(Kd);
  // Serial.print("\t");
  // Serial.println(basespeed);

  // Send debugging information to Bluetooth
//  if (counter % 10 == 0) {
  Serial.print("E ");
  Serial.print(error);
  // Serial.print(" ");
  Serial.print(" L ");
  Serial.print(leftMotorSpeed);
  // Serial.print(" ");
  Serial.print(" R ");
  Serial.print(rightMotorSpeed);
  Serial.print("  Kp ");
  Serial.print(Kp);
  Serial.print("  Kd");
  Serial.println(Kd);
//  }
  // for (int i = 0; i < )
  
  
  
//   if (Serial.available()) {
//     String command = Serial.readStringUntil('\n');
//     handleCommand(command);
//   }// Add delay to avoid overloading the motors
// }


}