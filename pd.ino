#include <QTRSensors.h>

// Motor and encoder pins
#define LEFT_MOTOR_PIN1 2
#define LEFT_MOTOR_PIN2 3
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 5
#define LEFT_ENCODER_PIN 18
#define RIGHT_ENCODER_PIN 19

// PD constants
#define KP 0.1
#define KD 0.5

// Sensor array
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// Motor control variables
int leftSpeed = 0;
int rightSpeed = 0;
const int baseSpeed = 150;

// Error variable
int lastError = 0;

void setup(){
  // Pin configurations
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  // Configure sensor array
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 31, 32, 33, 34, 35, 36, 37}, SensorCount);
  
  calibrateSensors();

  Serial.begin(9600);
}

void loop(){
  // Read sensor values
  uint16_t position = qtr.readLineWhite(sensorValues);

  // PD calculation
  int error = position - 3500;
  int derivative = error - lastError;
  int motorSpeed = KP * error + KD * derivative;

  leftSpeed = baseSpeed + motorSpeed;
  rightSpeed = baseSpeed - motorSpeed;

  leftSpeed = constrain(leftSpeed, 50, 255);
  rightSpeed = constrain(rightSpeed, 50, 255);

  // Always use PID-controlled motor speeds
  driveMotors(leftSpeed, rightSpeed);

  lastError = error;
}

void calibrateSensors(){
  for (uint16_t i = 0; i < 400; i++){
    qtr.calibrate();
    delay(20);
  }
}

void driveMotors(int left, int right){
  analogWrite(LEFT_MOTOR_PIN1, left);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, right);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}
