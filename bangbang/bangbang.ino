// sensor stuff
#define sensorCount 8
const int sensorPins[sensorCount] ={15, 2, 4, 16, 5, 13, 12, 14};
int sensorValues[sensorCount];

// motor stuff
#define leftMotor1 17
#define leftMotor2 18

#define rightMotor1 19
#define rightMotor2 21

#define leftMotorPWM 33
#define rightMotorPWM 32

#define baseSpeed 150

void setup() {
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

void loop() {
  // put your main code here, to run repeatedly:
  read();
  
  driveMotors(baseSpeed, baseSpeed, 0, 0);
  stopMotors();
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
