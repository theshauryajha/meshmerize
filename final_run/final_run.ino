#include <BluetoothSerial.h>
#include <QTRSensors.h>

// Define motor control pins
#define LH 19 // IN1
#define LL 23 // IN2
#define RH 21 // IN3
#define RL 2 // IN4

#define LED_PIN 14 // pin for LED to glow when Maze is solved

// Pololu sensor setup
QTRSensors qtrrc;
#define sensorCount 12
uint16_t sensorValues[sensorCount];  // Array to store sensor values
bool left, straight, right;
#define threshold 500 // Threshold for white line

// PID constants
float Kp = 0.035; // Proportional gain
float Ki = 0.0;   // Integral gain
float Kd = 0.044; // Derivative gain
int integral = 0;
int lastError = 0;

// Motor control variables
int baseSpeed = 100; // Base motor speed
int maxSpeed = 200;  // Maximum motor speed

// Bluetooth data
char myData[30] = { 0 }, s1[10], s2[10], s3[10], s4[10];

// Weighted multipliers
int leftBias[] = { -216, -125, -64, -9, -4, -1, 1, 4, 9, 16, 25, 36 };
int straightBias[] = { -36, -25, -16, -9, -4, -1, 1, 4, 8, 16, 25, 36 };
int rightBias[] = { -36, -25, -16, -9, -4, -1, 1, 4, 9, 64, 125, 216 };

// Path string and iterable
char path[] = "SRLRS";
int count = 0;

void setup() {
  // Declare motor pins as outputs
  pinMode(LH, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(RH, OUTPUT);
  pinMode(RL, OUTPUT);

  // Declare LED_PIN as output and set it to be off at the beginning
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize sensor array
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){ 13, 15, 12, 4, 27, 16, 26, 17, 25, 5, 33, 18 }, 12);
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

  // Check for incoming Bluetooth Serial data
  if (Serial.available()) {
    byte m = Serial.readBytesUntil('\n', myData, 20);
    myData[m] = '\0';  // null-byte
    if (sscanf(myData, "%[^','],%[^','],%[^','],%[^','],%s", s1, s2, s4, s3) == 4) {
      Kp = atof(s1);
      Kd = atof(s2);
      maxSpeed = atof(s3);
      baseSpeed = atof(s4);
      Serial.println("values updated");

    }
    else Serial.println("error in input!");
  }

  read();
  int position = calculatePosition(straightBias);
  int intersection = checkIntersection();

  /*if (intersection == 1){
    // There is some sort of intersection -> Make decision where to go based on path string
    switch(path[count++]){
      case 'L':
        position = calculatePosition(leftBias);
        break;
      case 'R':
        position = calculatePosition(rightBias);
        break;
      default:
        positon = calculateBias(straightBias);
        break;
    }
  }

  else if (intersection == -1){
    // end of maze
    while true{
      digitalWrite(LED_PIN, HIGH);
      stopMotors();
    }
  }*/

  // else (intersection == 0) implies no intersection, continue normal PID Control
  Serial.println(intersection);

  // Desired position = 0
  int error = position;
  integral += error;                                        // Calculate integral component
  int derivative = error - lastError;                       // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;                                        // Store current error as lastError for next iteration

  // Adjust motor speeds based on PID value
  int leftMotorSpeed = baseSpeed + turn;
  int rightMotorSpeed = baseSpeed - turn;

  // Ensure motor speeds are within bounds
  if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
  if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
  // if (leftMotorSpeed < 0 && leftMotorSpeed > -70) leftMotorSpeed = 0;
  // if (rightMotorSpeed < 0  && rightMotorSpeed > -70) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = -baseSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = -baseSpeed;

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

  // Debug sensor values data
  // if (counter % 10 == 0) {
  /*for (int i = 0; i < sensorCount; i++) {
    Serial.print(sensorValues[i] > threshold ? 1 : 0);
    Serial.print('\t');
  }*/
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
  // Serial.println(baseSpeed);

  // Send debugging information to Bluetooth
  //  if (counter % 10 == 0) {
  // Serial.print("E ");
  // Serial.print(error);
  // // Serial.print(" ");
  // Serial.print(" L ");
  // Serial.print(leftMotorSpeed);
  // // Serial.print(" ");
  // Serial.print(" R ");
  // Serial.print(rightMotorSpeed);
  // Serial.print("  Kp ");
  // Serial.print(Kp);
  // Serial.print("  Kd");
  // Serial.println(Kd);
  //  }
  // for (int i = 0; i < )

  //   if (Serial.available()) {
  //     String command = Serial.readStringUntil('\n');
  //     handleCommand(command);
  //   }// Add delay to avoid overloading the motors
  // }
}

void stopMotors(){
  analogWrite(LH, 0);
  analogWrite(LL, 0);
  analogWrite(RH, 0);
  analogWrite(RL, 0);
}

int checkIntersection(){
  if (left && right){
    extraInch();
    if (left && straight && right)
      return -1; // maze end
    else if (!left && straight && !right)
      return 1; // 4 - way
    else{
      extraInchBack();
      return 1; // LR T-junc
    }
  }
  else if (left && straight){
    extraInch();
    if (straight) return 1; // LS T-junc
    else{
      extraInchBack();
      return 0; // no intersection
    }
  }
  else if (straight && right){
    extraInch();
    if (straight) return 1; // RS T-junc
    else{
      extraInchBack();
      return 0; // no intersection
    }
  }
  return 0; // no intersection
}

int calculatePosition(int multipliers[]){
  int weightedSum = 0, totalSum = 0;
  for (int i = 0; i < sensorCount; i++){
    weightedSum += multipliers[i] * sensorValues[i];
    totalSum += sensorValues[i];
  }

  int position = 1000 * weightedSum / totalSum;
  return position;
}

void extraInch(){
  analogWrite(LH, baseSpeed);
  analogWrite(LL, 0);
  analogWrite(RH, baseSpeed);
  analogWrite(RL, 0);
  delay(80);
  stopMotors();

  read();
}

void extraInchBack(){
  analogWrite(LH, 0);
  analogWrite(LL, baseSpeed);
  analogWrite(RH, 0);
  analogWrite(RL, baseSpeed);
  delay(80);
  stopMotors();

  read();
}

void read(){
  qtrrc.readLineWhite(sensorValues);
  
  int totalSum = 0;
  sensorValues[0] = 1000 - sensorValues[0];
  sensorValues[11] = 1000 - sensorValues[11];
  for (int i = 1; i < sensorCount-1; i++){
    sensorValues[i] = 1000 - sensorValues[i];
    totalSum += sensorValues[i];
  }
  straight = (totalSum != 0);
  
  left = sensorValues[0] > threshold;
  right = sensorValues[11] > threshold;
}