#include <BluetoothSerial.h>
#include <QTRSensors.h>

// Define motor control pins
#define LH 19 // IN1
#define LL 23 // IN2
#define RH 21 // IN3
#define RL 2 // IN4

#define LED_PIN 12 // pin for LED to glow when Maze is solved

// Pololu sensor setup
QTRSensors qtrrc;
#define sensorCount 12
uint16_t sensorValues[sensorCount];  // Array to store sensor values
#define threshold 500; // Threshold for white line

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
int leftMultiplier[] = { 0, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0 };
int straightMultiplier[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
int rightMultiplier[] = { 0, 0, 0, 0, 0, 5, 6, 7, 8, 9, 10, 11 };

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

  // Control Loop
  qtrrc.read(sensorValues); // Read sensor data
  for (int i = 0; i < sensorCount; i++){
    // Incoming values are in the range 0 to 2500
    map(sensorValues[i], 0, 2500, 0, 1000); // Scale to range 0 to 1000
  }

  int position;
  if (intersection() == 1) {
    // If there is an intersection, check the path array for decision and act on it
    if (path[count] == 'L')
      leftPID();
    else if (path[count] == 'R')
      rightPID();
    else
      straightPID();
    count++;
    delay(100);
  }
  // If the end of the maze is found, stop the bot
  else if (intersection() == -1){
    digitalWrite(LED_PIN, HIGH); // Turn on LED to indicate Maze is solved
    stop(); // Stop the bot
    while (true) {} // Infinite loop with no control signals, bot stays stopped
  }

  // Continue PID Controlled Line Following
  position = qtrrc.readLineWhite(sensorValues);

  // Compute error (desired position is 5500 for 12-sensor array)
  int error = position - 5500;
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
  for (int i = 0; i < 12; i++) {
    Serial.print(sensorValues[i] > threshold ? 1 : 0);
    Serial.print('\t');
  }
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

int weightedRead(int multipliers[]) {
  int weightedSum = 0, totalSum = 0;
  for (int i = 0; i < sensorCount; i++) {
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
}

void stop(){
  analogWrite(LH, 0);
  analogWrite(LL, 0);
  analogWrite(RH, 0);
  analogWrite(RL, 0);
}

int intersection(){
  bool left = sensorValues[0] > 500;
  bool straight = sensorValues[5] > 500 && sensorValues[6] > 500;
  bool right = sensorValues[11] > 500;

  // Debugging
  Serial.print("L");
  Serial.print(left);
  Serial.print("S");
  Serial.print(straight);
  Serial.print("R");
  Serial.print(right);

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
  analogWrite(LH, baseSpeed);
  analogWrite(LL, 0);
  analogWrite(RH, baseSpeed);
  analogWrite(RL, 0);
  delay(10);
}

void extraInchBack(){
  analogWrite(LH, 0);
  analogWrite(LL, baseSpeed);
  analogWrite(RH, 0);
  analogWrite(RL, baseSpeed);
  delay(10);
}
