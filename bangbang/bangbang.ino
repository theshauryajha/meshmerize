// define motor pins
#define leftMotor1 2
#define leftMotor2 3
#define leftMotorPWM 6
#define rightMotor1 4
#define rightMotor2 5
#define rightMotorPWM 7
#define STDBY 8

// define sensor pins
#define leftMostSensor 15
#define leftCenterSensor 14
#define centerSensor 13
#define rightCenterSensor 12
#define rightMostSensor 11

// threshold to detect white line
#define th 100

// speed control variables
#define baseSpeed 120

// variables for sensor values
int sensor1, sensor2, sensor3, sensor4, sensor5;

void setup(){
  // setup sensor pins as input
  pinMode(leftMostSensor, INPUT);
  pinMode(leftCenterSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightMostSensor, INPUT);

  // setup motor pins as output
  pinMode(leftMotor1 ,OUTPUT);
  pinMode(leftMotor2 ,OUTPUT);
  pinMode(leftMotorPWM ,OUTPUT);
  pinMode(rightMotor1 ,OUTPUT);
  pinMode(rightMotor2 ,OUTPUT);
  pinMode(rightMotorPWM ,OUTPUT);

  // setup standby pin, initally high, low to kill motors
  pinMode(STDBY ,OUTPUT);
  digitalWrite(STDBY, HIGH);
}

void loop(){
  // read sensor values and store
  readSensorValues();
  
  // drive motors according to sensor values
  if (sensor1 > th && sensor2 < th && sensor3 < th && sensor4 < th && sensor5 > th){
    digitalWrite(STDBY, HIGH);
    driveMotors(baseSpeed, baseSpeed);
  }
  else
    digitalWrite(STDBY, LOW);
}

void readSensorValues(){
  sensor1 = analogRead(leftMostSensor);
  sensor2 = analogRead(leftCenterSensor);
  sensor3 = analogRead(centerSensor);
  sensor4 = analogRead(rightCenterSensor);
  sensor5 = analogRead(rightMostSensor);
}

void driveMotors(int left, int right){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  analogWrite(leftMotorPWM, left);
  analogWrite(rightMotorPWM, right);
}
