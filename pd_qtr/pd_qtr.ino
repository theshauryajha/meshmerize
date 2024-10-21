#include <QTRSensors.h>
#include "BluetoothSerial.h"

//Bluetooth Properties
BluetoothSerial SerialBT;
String message = "";
String data = "";

//PD control properties
int kp = 0; //proportional gain
int kd = 0; //derivative
int lastError = 0; //check previous error to calculate error difference
int goal = 5500;

//right motor properties
int RIGHT_MOTOR_3 = 32; //IN3
int RIGHT_MOTOR_4 = 35; //IN4
int RIGHT_PWM = 34;

//left motor properties
int LEFT_MOTOR_1 = 25; //IN1
int LEFT_MOTOR_2 = 33; //IN2
int LEFT_PWM = 26;

//general motor properties
int BASE_SPEED = 150;
int motorPins[6] = {RIGHT_MOTOR_3, RIGHT_MOTOR_4, RIGHT_PWM, LEFT_MOTOR_1, LEFT_MOTOR_2, LEFT_PWM};

QTRSensors qtr;

const uint8_t SensorCount = 12;
uint16_t sensorValues[SensorCount];
int led_pin = 10;

void setup() {
  // put your setup code here, to run once:
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){}, SensorCount);
  pinMode(led_pin, OUTPUT);
  for(int i = 0; i < 6; i++){
    pinMode(motorPins[i], OUTPUT);
  }
  calibrate_sensor();
  Serial.begin(115200);
  SerialBT.begin("My_ESP");
}

void loop() {

  //find position of white line on black background
  int position = qtr.readLineWhite(sensorValues);

  //calculate error
  int error = goal - position;

  //calculate adjustment
  int adjustment = kp*error + kd*(error-lastError);

  int rightMotorSpeed = constrain(BASE_SPEED + adjustment, 50, 255);
  int leftMotorSpeed = constrain(BASE_SPEED - adjustment, 50, 255);

  driveMotors(rightMotorSpeed, leftMotorSpeed, 0, 0);

  lastError = error;
}

void calibrate_sensor(){
  digitalWrite(led_pin, HIGH);
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(led_pin, LOW);
}

void driveRight(int rightPWM, int mode){ //mode 0 means clockwise, mode 1 means anti clockwise
  switch(mode){
    case 0:
    digitalWrite(RIGHT_MOTOR_3, HIGH);
    digitalWrite(RIGHT_MOTOR_4, LOW);
    break;
    case 1:
    digitalWrite(RIGHT_MOTOR_3, LOW);
    digitalWrite(RIGHT_MOTOR_4, HIGH);
    break;
  }

  analogWrite(RIGHT_PWM, rightPWM);
}

void driveLeft(int leftPWM, int mode){ //mode 0 means clockwise, mode 1 means anti clockwise
  switch(mode){
    case 0:
    digitalWrite(LEFT_MOTOR_1, HIGH);
    digitalWrite(LEFT_MOTOR_2, LOW);
    break;
    case 1:
    digitalWrite(LEFT_MOTOR_1, LOW);
    digitalWrite(LEFT_MOTOR_2, HIGH);
    break;
  }

  analogWrite(LEFT_PWM, leftPWM);
}

void driveMotors(int rightPWM, int leftPWM, int rightMode, int leftMode){
  driveRight(rightPWM, rightMode);
  driveLeft(leftPWM, leftMode);
}

void stopMotors(){
  digitalWrite(RIGHT_MOTOR_3, LOW);
  digitalWrite(RIGHT_MOTOR_4, LOW);
  digitalWrite(LEFT_MOTOR_1, LOW);
  digitalWrite(LEFT_MOTOR_2, LOW);
}

void findKpKd(){
  if(SerialBT.available()){
    char inComing_char = SerialBT.read();
    if (inComing_char != "\n"){
      message += String(inComing_char);
    }
    else{
      message = "";
    }
  }

  if(message == "kp"){
    if(SerialBT.available()){
      char inComing_kp = SerialBT.read();
      if(inComing_kp != "\n"){
        data += String(inComing_kp);
      }
      else{
        data = "";
      }
    }
    kp = data.toInt();
  }

  if(message == "kd"){
    if(SerialBT.available()){
      char inComing_kd = SerialBT.read();
      if(inComing_kd != "\n"){
        data += String(inComing_kp);
      }
      else{
        data = "";
      }
    }
    kd = data.toInt();
  }
}
