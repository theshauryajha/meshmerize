#include <BluetoothSerial.h>
#include <QTRSensors.h>

int final_arr[2] = {0,0};
//led 14
// 34 35 switch
// in1  - 19 
// in 2 - 23
// in 3 - 21
// in 4 - 2
#define RH 21  // Right motor control
#define LH 19  // Left motor control
#define RL   2 // Reverse Right motor control
#define LL 23 // Reverse Left motor control
#define led 14


// Pololu sensor setup
QTRSensors qtrrc;
int basespeed = 75; // Base motor speed
int maxSpeed = 100; // Maximum motor speed
uint16_t sensorValues[12]; // Array to store sensor values

// PID constants
float Kp = 0.005;  // Proportional gain
float Ki = 0.0;  // Integral gain......

float Kd = 0.05;  // Derivative gain
int integral = 0;
int lastError = 0;
char myData[30] = { 0 } , s1[10], s2[10],s3[10],s4[10];

void setup() {
  // Set motor pins as outputs
  pinMode(RH , OUTPUT);
  pinMode(RL , OUTPUT);
  pinMode(LL , OUTPUT);
  pinMode(LH , OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Initialize sensor array
  qtrrc.setTypeRC();
  qtrrc.setSensorPins((const uint8_t[]){   13, 15, 12, 4, 27, 16, 26, 17, 25, 5, 33, 18  }, 12);
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
  
    if (Serial.available()){
    byte m = Serial.readBytesUntil('\n', myData, 20);
    myData[m] = '\0';  //null-byte
    if (sscanf(myData, "%[^','],%[^','],%[^','],%[^','],%s", s1, s2,s4,s3) == 4) {
      Kp = atof(s1);
      Kd = atof(s2);
      maxSpeed = atof(s3);
      basespeed=atof(s4);
      Serial.println("values updated");
      
    } else Serial.println("error in input!");
  }


  readLFSsensors();

  int error = final_arr[1];
  int mode = final_arr[0];
  // Serial.print("E");
  // Serial.print(error);
  // Serial.print("M");
  // Serial.print(mode);

  switch(mode){
    case 1: //right turn
      runExtraInch();
      readLFSsensors();
      if(final_arr[0] == 3){  //if there is no line
        goAndTurn(3);
      }
      break;

    case 2: //left turn
      goAndTurn(2);
      break;

    case 3: //no line
      stopMotors();
      goAndTurn(2);
      goAndTurn(2);
      break;

    case 4: //following line
      PID(error);
      break;

    case 5: //horizontal line
      runExtraInch();
      readLFSsensors();
      if(final_arr[0] == 5){  //check if end of maze or not
        mazeEnd();
      }
      else{
        goAndTurn(2);
      }
      break;
  }


//if (counter % 10 == 0) {
  for (int i = 0; i < 12; i++) {
    Serial.print(sensorValues[i] > 500 ? 1 : 0);
    // Serial.print(" ");
  }
  Serial.print(" ");

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

void readLFSsensors(){

  int position = qtrrc.readLineWhite(sensorValues);

  int mode;
  int error;

  int s = 0;
  for (int i = 1; i < 11; i++)
    s += sensorValues[i];
  Serial.print(s);

  if(sensorValues[0] > 500){
    mode = 1; //RIGHT TURN
    error = 0;
  }
  if(sensorValues[11] > 500){
    mode = 2; //LEFT TURN
    error = 0;
  }
  if(sensorValues[0] < 500 && sensorValues[11] < 500 && s == 0){
    mode = 3; //NO LINE
    error = 0;
  }
  if(sensorValues[0] < 500 && sensorValues[11] < 500 && s != 0){
    mode = 4; //FOLLOWING LINE
    error = position - 5500;
  }
  if(sensorValues[0] > 500 && sensorValues[11] > 500 && s != 0){
    mode = 5; //HORIZONTAL LINE
    error = 0;
  }
  final_arr[0] = mode;
  final_arr[1] = error;
}

void PID(int error){
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
}

void runExtraInch(){
  lastError = 0;
  PID(0);
  delay(20);
  stopMotors();
}

void goAndTurn(int mode){
  switch(mode){
    case 2:
      analogWrite(LH, 0);
      analogWrite(LL, basespeed);
      analogWrite(RH, basespeed);
      analogWrite(RL, 0);
      break;
    case 3:
      analogWrite(LL, 0);
      analogWrite(LH, basespeed);
      analogWrite(RL, basespeed);
      analogWrite(RH, 0);
      break;
  }
  delay(30);
}
void mazeEnd(){
  while(true){
    stopMotors();
    digitalWrite(led, HIGH);
  }

}
void stopMotors(){
  analogWrite(LL, 0);
  analogWrite(LH, 0);
  analogWrite(RL, 0);
  analogWrite(RH, 0);
  delay(20);
}
