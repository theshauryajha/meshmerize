#include <QTRSensors.h>

// Motor and encoder pins
//left motor pins
#define LEFT_MOTOR_PIN1 2
#define LEFT_MOTOR_PIN2 3
#define LEFT_MOTOR_PWM 1 //dummy pin value
#define LEFT_ENCODER_PIN 18

//right motor pins
#define RIGHT_MOTOR_PIN1 4
#define RIGHT_MOTOR_PIN2 5
#define RIGHT_MOTOR_PWM 10 //dummy pin value
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

char path[100];
int pathLength = 0;

// Threshold to detect white line
const int threshold = 900;

#define MAX_VISITS 100
struct Intersection{
  int x, y;  
  char direction;  
};

Intersection visited[MAX_VISITS];
int visitCount = 0;
int posX = 0, posY = 0;  e
char currentDirection = 'N';

void setup(){  
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);
  
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){30, 31, 32, 33, 34, 35, 36, 37}, SensorCount);

  calibrateSensors();
  Serial.begin(9600);
}

void loop(){
  uint16_t position = qtr.readLineWhite(sensorValues);

  int error = position - 3500;
  int derivative = error - lastError;
  int motorSpeed = KP * error + KD * derivative;

  leftSpeed = baseSpeed + motorSpeed;
  rightSpeed = baseSpeed - motorSpeed;
  
  leftSpeed = constrain(leftSpeed, 50, 255);
  rightSpeed = constrain(rightSpeed, 50, 255);

  // Check for intersection
  int intersectionType = checkIntersection();
  if (intersectionType != 1){
    handleIntersection(intersectionType);
  }

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

int checkIntersection(){
  bool left = sensorValues[0] > threshold;
  bool right = sensorValues[7] > threshold;
  bool straight = (sensorValues[3] > threshold || sensorValues[4] > threshold);
  
  if (left && right && straight) return 7;  // 4-way intersection
  if (left && straight) return 6;
  if (right && straight) return 5;
  if (left && right) return 4; // T junction
  if (left) return 3;
  if (right) return 2;
  if (straight) return 1; // no intersection
  if (!left && !right && !straight) return 0; // dead end
}

void handleIntersection(int intersectionType){
  // Slow down or stop briefly
  int currentLeft = leftSpeed;
  int currentRight = rightSpeed;
  driveMotors(currentLeft / 2, currentRight / 2);
  delay(50);

  if (checkVisitedIntersection()){
    // If the intersection was visited before, take an alternate path or backtrack
    backtrackOrAlternate();
  }
  else{
    switch (intersectionType){
      case 7:
      case 6:
      case 4:
      case 3:
        turnLeft();
        path[pathLength++] = 'L';
        updatePosition('L');
        break;
      case 2:
        turnRight();
        path[pathLength++] = 'R';
        updatePosition('R');
        break;
      case 5:
      case 1:  // Straight path only
        // Continue straight
        path[pathLength++] = 'S';
        updatePosition('S');
        break;
      case 0:  // Dead end
        turnAround();
        path[pathLength++] = 'U';
        updatePosition('U');
        break;
    }
  }
}

bool checkVisitedIntersection(){
  // Check if this intersection was visited before
  for (int i = 0; i < visitCount; i++){
    if (visited[i].x == posX && visited[i].y == posY && visited[i].direction == currentDirection){
      return true;
    }
  }
  return false;
}

void backtrackOrAlternate(){
  // Decide to either backtrack or take a different path
  if (currentDirection == 'L'){
    turnRight();
    path[pathLength++] = 'R';
  } else if (currentDirection == 'R'){
    turnLeft();
    path[pathLength++] = 'L';
  } else{
    turnAround();
    path[pathLength++] = 'U';
  }
  updatePosition(path[pathLength - 1]);
}

void turnLeft(){
  while (sensorValues[0] > threshold || sensorValues[1] > threshold){
    qtr.readLineWhite(sensorValues);
    int error = sensorValues[0] - sensorValues[7];
    int derivative = error - lastError;
    int motorSpeed = KP * error + KD * derivative;
    
    leftSpeed = -baseSpeed + motorSpeed;
    rightSpeed = baseSpeed + motorSpeed;
    
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    driveMotors(leftSpeed, rightSpeed);
    
    lastError = error;
  }
}

void turnRight(){
  while (sensorValues[7] > threshold || sensorValues[6] > threshold){
    qtr.readLineWhite(sensorValues);
    int error = sensorValues[7] - sensorValues[0];
    int derivative = error - lastError;
    int motorSpeed = KP * error + KD * derivative;
    
    leftSpeed = baseSpeed + motorSpeed;
    rightSpeed = -baseSpeed + motorSpeed;
    
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);
    
    driveMotors(leftSpeed, rightSpeed);
    
    lastError = error;
  }
}

void turnAround(){
  // First, turn right until we lose the line
  while (sensorValues[3] > threshold || sensorValues[4] > threshold){
    qtr.readLineWhite(sensorValues);
    driveMotors(baseSpeed, -baseSpeed);
  }
  // Then, turn right until we find the line again
  while (sensorValues[3] < threshold && sensorValues[4] < threshold){
    qtr.readLineWhite(sensorValues);
    driveMotors(baseSpeed, -baseSpeed);
  }
}

void updatePosition(char move){
  if (move == 'L'){
    if (currentDirection == 'N') posX--;
    else if (currentDirection == 'E') posY++;
    else if (currentDirection == 'S') posX++;
    else posY--;
    currentDirection = 'W';
  } else if (move == 'R'){
    if (currentDirection == 'N') posX++;
    else if (currentDirection == 'E') posY--;
    else if (currentDirection == 'S') posX--;
    else posY++;
    currentDirection = 'E';
  } else if (move == 'S'){
    if (currentDirection == 'N') posY++;
    else if (currentDirection == 'E') posX++;
    else if (currentDirection == 'S') posY--;
    else posX--;
  } else if (move == 'U'){
    // U-turn changes direction but keeps position unchanged
    if (currentDirection == 'N') currentDirection = 'S';
    else if (currentDirection == 'S') currentDirection = 'N';
    else if (currentDirection == 'E') currentDirection = 'W';
    else currentDirection = 'E';
  }

  // Record this intersection as visited
  visited[visitCount].x = posX;
  visited[visitCount].y = posY;
  visited[visitCount].direction = currentDirection;
  visitCount++;
}
