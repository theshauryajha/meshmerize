#include <BluetoothSerial.h>
#include <QTRSensors.h>

// in1  - 19
// in 2 - 23
// in 3 - 21
// in 4 - 2
#define RH 21  // Right motor control
#define LH 19  // Left motor control
#define RL 2   // Reverse Right motor control
#define LL 23  // Reverse Left motor control

#define LED_PIN 12

// Pololu sensor setup
QTRSensors qtrrc;
int basespeed = 100;         // Base motor speed
int maxSpeed = 200;         // Maximum motor speed
uint16_t sensorValues[12];  // Array to store sensor values

// PID constants
float Kp = 0.035;  // Proportional gain
float Ki = 0.0;    // Integral gain
float Kd = 0.044;   // Derivative gain
int integral = 0;
int lastError = 0;
char myData[30] = { 0 }, s1[10], s2[10], s3[10], s4[10];

int leftMultiplier[] = { 0, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0 };
int straightMultiplier[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
int rightMultiplier[] = { 0, 0, 0, 0, 0, 5, 6, 7, 8, 9, 10, 11 };


int count = 0;
char path[] = "SRLRS";

void setup() {
  // Set motor pins as outputs
  pinMode(RH, OUTPUT);
  pinMode(RL, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(LH, OUTPUT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize sensor array
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){ 13, 15, 14, 4, 27, 16, 26, 17, 25, 5, 33, 18 }, 12);
  Serial.begin(9600);
  // Serial.begin("PID_Tuner"); // Bluetooth device name

  Serial.println("Bluetooth Device is Ready to Pair");
  // Serial.println("Bluetooth Device is Ready to Pair");

  // Calibrate sensors
  for (int i = 0; i < 250; i++) {
    qtrrc.calibrate();
    delay(20);
    Serial.println("Calibrating...");
    //Serial.println("Calibrating...");
  }
}

// int counter = 0;
void loop() {
  Serial.println("");
  // counter += 1;
  // Check for incoming Bluetooth Serial data

  if (Serial.available()) {
    byte m = Serial.readBytesUntil('\n', myData, 20);
    myData[m] = '\0';  //null-byte
    if (sscanf(myData, "%[^','],%[^','],%[^','],%[^','],%s", s1, s2, s4, s3) == 4) {
      Kp = atof(s1);
      Kd = atof(s2);
      maxSpeed = atof(s3);
      basespeed = atof(s4);
      Serial.println("values updated");

    } else Serial.println("error in input!");
  }

  qtrrc.read(sensorValues);
  for (int i = 0; i < 12; i++){
    map(sensorValues[i], 0, 2500, 0, 1000);
  }

  int position;
  if (intersection() == 1) {
    if (path[count] == 'L')
      leftPID();
    else if (path[count] == 'R')
      rightPID();
    else
      straightPID();
    count++;
    delay(100);
  }
  else if (intersection() == -1)
    stop();

  position = qtrrc.readLineWhite(sensorValues);

  
  /*for (int i = 0; i < 12; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }*/


  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
  integral += error;                                        // Calculate integral component
  int derivative = error - lastError;                       // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;                                        // Store current error as lastError for next iteration

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


int weightedRead(int multipliers[]) {
  int weightedSum = 0, totalSum = 0;
  for (int i = 0; i < 12; i++) {
    weightedSum += multipliers[i] * sensorValues[i];
    totalSum += sensorValues[i];
  }

  int position = 1000 * weightedSum / totalSum;
  return position;
}

void leftPID(){
  int position = weightedRead(leftMultiplier);
  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
  integral += error;                                        // Calculate integral component
  int derivative = error - lastError;                       // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;                                        // Store current error as lastError for next iteration

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
}

void rightPID(){
  int position = weightedRead(rightMultiplier);
  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
  integral += error;                                        // Calculate integral component
  int derivative = error - lastError;                       // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;                                        // Store current error as lastError for next iteration

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
}

void straightPID(){
  int position = weightedRead(straightMultiplier);
  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
  integral += error;                                        // Calculate integral component
  int derivative = error - lastError;                       // Calculate derivative component
  int turn = Kp * error + Ki * integral + Kd * derivative;  // Calculate PID value
  lastError = error;                                        // Store current error as lastError for next iteration

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
}

void stop(){
  analogWrite(LH, 0);
  analogWrite(LL, 0);
  analogWrite(RH, 0);
  analogWrite(RL, 0);

  digitalWrite(LED_PIN, HIGH);
}

int intersection(){
  bool left = sensorValues[0] > 500;
  bool straight = sensorValues[5] > 500 && sensorValues[6] > 500;
  bool right = sensorValues[11] > 500;

  Serial.print("L");
  Serial.print(left);
  Serial.print("R");
  Serial.print(right);
  Serial.print("S");
  Serial.print(straight);

  if (left && right){
    stop();
    delay(1000);
    extraInch();
    stop();
    delay(1000);
    if (left && straight && right)
      return -1; // maze end
    else if (!left && straight && !right)
      return 1; // 4 - way
    else{
      extraInchBack();
      stop();
      delay(1000);
      return 1; // LR T-junc
    }
  }
  else if (left && straight){
    stop();
    delay(1000);
    extraInch();
    stop();
    delay(1000);
    if (straight) return 1; // LS T-junc
    else{
      extraInchBack();
      stop();
      delay(1000);
      return 0; // no intersection
    }
  }
  else if (straight && right){
    stop();
    delay(1000);
    extraInch();
    stop();
    delay(1000);
    if (straight) return 1; // RS T-junc
    else{
      extraInchBack();
      stop();
      delay(1000);
      return 0; // no intersection
    }
  }
  return 0; // no intersection
}

void extraInch(){
  analogWrite(LH, basespeed);
  analogWrite(LL, 0);
  analogWrite(RH, basespeed);
  analogWrite(RL, 0);
  delay(10);
}

void extraInchBack(){
  analogWrite(LH, 0);
  analogWrite(LL, basespeed);
  analogWrite(RH, 0);
  analogWrite(RL, basespeed);
  delay(10);
}
