// Define motor control pins
#define LH 19 // IN1
#define LL 23 // IN2
#define RH 21 // IN3
#define RL 2 // IN4

#define baseSpeed 100; // Base motor speed

void setup(){
  // put your setup code here, to run once:
  // Declare motor pins as outputs
  pinMode(LH, OUTPUT);
  pinMode(LL, OUTPUT);
  pinMode(RH, OUTPUT);
  pinMode(RL, OUTPUT);
}

void loop(){
  // put your main code here, to run repeatedly:
  driveMotors(baseSpeed, baseSpeed, 0, 0);
  delay(2000); // run motors for 2 seconds
  stopMotors();
  delay(2000);
}

void driveMotors(){
  analogWrite(LH, baseSpeed);
  analogWrite(LL, 0);
  analogWrite(RH, baseSpeed);
  analogWrite(RL, 0);
}

void stopMotors(){
  analogWrite(LH, 0);
  analogWrite(LL, 0);
  analogWrite(RH, 0);
  analogWrite(RL, 0);
}