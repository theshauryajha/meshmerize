#define leftMotor1 2
#define leftMotor2 3
#define leftMotorPWM 6
#define rightMotor1 4
#define rightMotor2 5
#define rightMotorPWM 7
#define STDBY 8

void setup(){
  // put your setup code here, to run once:
  pinMode(leftMotor1 ,OUTPUT);
  pinMode(leftMotor2 ,OUTPUT);
  pinMode(leftMotorPWM ,OUTPUT);
  pinMode(rightMotor1 ,OUTPUT);
  pinMode(rightMotor2 ,OUTPUT);
  pinMode(rightMotorPWM ,OUTPUT);

  digitalWrite(STDBY, HIGH);
}

void loop(){
  // put your main code here, to run repeatedly:
  driveMotorRight(120);
  driveMotorLeft(120);

  delay(3000); // run for 3 seconds
  digitalWrite(STDBY, LOW);
}

void driveMotorRight(int right){
  digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, LOW);

  analogWrite(rightMotorPWM, right);
}

void driveMotorLeft(int left){
  digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);

  analogWrite(leftMotorPWM, left);
}
