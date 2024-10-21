#define leftMotor1 2
#define leftMotor2 4

#define rightMotor1 17
#define rightMotor2 5

#define rightMotorPWM 16
#define leftMotorPWM 18

#define baseSpeed 150

void setup(){
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
}

void loop(){
  driveMotors(baseSpeed, baseSpeed, 0, 0);
  stopMotors();
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

void driveMotors(int rightPWM, int leftPWM, int rightMode, int leftMode){
  driveRight(rightPWM, rightMode);
  driveLeft(leftPWM, leftMode);
}

void stopMotors(){
  digitalWrite(rightMotor1, LOW);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(leftMotor1, LOW);
  digitalWrite(leftMotor2, LOW);
}
