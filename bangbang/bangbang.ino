//sensor stuff
#define sensorCount 4
const int sensorPins[sensorCount] = {15, 2, 4, 16};
int sensorValues[sensorCount];

// motor stuff
#define leftMotor1 17
#define leftMotor2 18

#define rightMotor1 19
#define rightMotor2 21

#define leftMotorPWM 33
#define rightMotorPWM 32

#define baseSpeed 120

// sensorValues combination
int straight[sensorCount] = {1, 0, 0, 1};
int left[sensorCount] = {0, 0, 0, 1};
int right[sensorCount] = {1, 0, 0, 0};

void setup(){
  // put your setup code here, to run once:
  // sensor stuff
  for (int i = 0; i < sensorCount; i++){
    pinMode(sensorPins[i], INPUT);
  }
  
  // motor stuff
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  read();

  if (comp(sensorValues, left)){
    while (!comp(sensorValues, straight)){
      driveMotors(baseSpeed, baseSpeed, 1, 0);
    }
  }

  else if (comp(sensorValues, right)){
    while (!comp(sensorValues, straight)){
      driveMotors(baseSpeed, baseSpeed, 0, 1);
    }
  }

  else{
    driveMotors(baseSpeed, baseSpeed, 0, 0);
  }
}

void read(){
  // 0 for white (line), 1 for black (no line)
  for (int i = 0; i < sensorCount; i++){
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}

bool comp(int arr1[sensorCount], int arr2[sensorCount]){
  for (int i = 0; i < sensorCount; i++){
    if (arr1[i] != arr2[i])
      return false;
  }
  return true;
}

void driveLeft(int leftPWM, int mode){ //mode 0 means clockwise, mode 1 means anti clockwise
  switch(mode){
    case 0:
      digitalWrite(leftMotor1, HIGH);
      digitalWrite(leftMotor2, LOW);
      break;
    case 1:
      digitalWrite(leftMotor1, LOW);
      digitalWrite(leftMotor2, HIGH);
      break;
  }

  analogWrite(leftMotorPWM, leftPWM);
}

void driveRight(int rightPWM, int mode){ //mode 0 means clockwise, mode 1 means anti clockwise
  switch(mode){
    case 0:
      digitalWrite(rightMotor1, HIGH);
      digitalWrite(rightMotor2, LOW);
      break;
    case 1:
      digitalWrite(rightMotor1, LOW);
      digitalWrite(rightMotor2, HIGH);
      break;
  }

  analogWrite(rightMotorPWM, rightPWM);
}

void driveMotors(int leftPWM, int rightPWM, int leftMode, int rightMode){
  driveLeft(leftPWM, leftMode);
  driveRight(rightPWM, rightMode);
}

void stopMotors(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
}
