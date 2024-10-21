//PD control properties
int kp = 0; //proportional gain
int kd = 0; //derivative
int lastError = 0; //check previous error to calculate error difference

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

// sensor array properties
int SensorPins[8] = {15, 2, 0, 4, 33, 32, 35, 34}; //left to right, A8 to A1

void setup() {
  // put your setup code here, to run once:
  for(int i = 0; i < 6; i++){
    pinMode(motorPins[i], OUTPUT); //set all motor pins to output;
  }

  for(int i = 0; i < 8; i++){
    pinMode(SensorPins[i], INPUT); //set sensor array pins to input
  }
  Serial.begin(9600); //begin serial monitor for debugging
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
  int position = readPosition(); //reading the position based on raw sensor values

/*  int goal = 3500; //this is the goal where error will be zero

  int error = goal - position; //calculate error, left side positive, right side negative

  int adjustment = kp*error + kd*(error - lastError); //calculate speed adjustment using PD

  int leftSpeed = constrain(BASE_SPEED - adjustment, 50, 255); //update the motor speed
  int rightSpeed = constrain(BASE_SPEED + adjustment, 50, 255);

  //drive the motors with the updated speed
  driveMotors(rightSpeed, leftSpeed);*/
}

int readPosition(){
  int sum_num = 0;  // weighted sum of sensor values
  int sum_den = 0;  // sum of sensor values
  for(int i = 0; i < 8; i++){
    //bring the range of values from 0-4095 to 0-1000 and invert the values so that 0 corresponds to black and 1000 corresponds to white
    int rawValue = 1000 - int (analogRead(SensorPins[i]) * 1000/4095);
    Serial.print(analogRead(SensorPins[i]));
    Serial.print("\t");

    //calculate weighted sum
    sum_num += rawValue * i * 1000;

    //calculate sum of values
    sum_den += rawValue;
  }
  Serial.println();

  //calculate position
  int position = int (sum_num/sum_den);
  return(position);
}

void driveMotors(int rightPWM, int leftPWM){
  driveLeft(leftPWM);
  driveRight(rightPWM);
}

void driveLeft(int leftPWM){
  digitalWrite(LEFT_MOTOR_1, HIGH); //set it to clockwise
  digitalWrite(LEFT_MOTOR_2, LOW);

  analogWrite(LEFT_PWM, leftPWM); //set speed
}

void driveRight(int rightPWM){
  digitalWrite(RIGHT_MOTOR_3, LOW); //set it to clockwise
  digitalWrite(RIGHT_MOTOR_4, HIGH);

  analogWrite(RIGHT_PWM, rightPWM); //set speed
}
