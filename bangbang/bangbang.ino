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

// speed control variables
#define baseSpeed 120

// array for sensor values
#define sensorCount 5
int sensorValues[sensorCount];

// define potential states of sensorValues
int straight[] = {0, 1, 1, 1, 0};
int slightLeft[] = {0, 1, 1, 0, 0};
int slightRight[] = {0, 0, 1, 1, 0};
int left[] = {0, 1, 0, 0, 0};
int right[] = {0, 0, 0, 1, 0};
int hardLeft[] = {1, 0, 0, 0, 0};
int hardRight[] = {0, 0, 0, 0, 1};

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

void loop() {
  // read sensor values and store
  readSensorValues();
}

void readSensorValues(){
  sensorValues[0] = digitalRead(leftMostSensor);
  sensorValues[1] = digitalRead(leftCenterSensor);
  sensorValues[2] = digitalRead(centerSensor);
  sensorValues[3] = digitalRead(rightCenterSensor);
  sensorValues[4] = digitalRead(rightMostSensor);
}

void driveMotors(int left, int right){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  analogWrite(leftMotorPWM, left);
  analogWrite(rightMotorPWM, right);
}
