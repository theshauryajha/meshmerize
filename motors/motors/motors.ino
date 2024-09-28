#define LEFT1 2
#define LEFT2 3
#define LEFT_PWM 6
#define RIGHT1 4
#define RIGHT2 5
#define RIGHT_PWM 7
#define STDBY 8

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT1, OUTPUT);
  pinMode(LEFT2, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT1, OUTPUT);
  pinMode(RIGHT2, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(STDBY, OUTPUT);

  digitalWrite(STDBY, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  driveMotorRight(150);
  driveMotorLeft(150);

  delay(3000);
  digitalWrite(STDBY, LOW);
}

void driveMotorRight(int right){
  digitalWrite(RIGHT1, HIGH);
  digitalWrite(RIGHT2, LOW);
  analogWrite(RIGHT_PWM, right);
}

void driveMotorLeft(int left){
  digitalWrite(LEFT1, HIGH);
  digitalWrite(LEFT2, LOW);
  analogWrite(LEFT_PWM, left);
}
