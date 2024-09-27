// PD Control Sample - We will have to adjust these later
float Kp = 2.0;
float Kd = 1.0;
int baseSpeed = 200;
int previousError = 0;
int maxSpeed = 255;

// Assuming Pins here
int motor1Pin1 = 9;
int motor2Pin1 = 10;
int motor1Pin2 = 11;
int motor2Pin2 = 12;
int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Sample Maze only
#define GRID_ROWS 5
#define GRID_COLS 5

int maze[GRID_ROWS][GRID_COLS] = {
  {0, 1, 0, 0, 0},
  {0, 1, 0, 1, 0},
  {0, 0, 0, 1, 0},
  {0, 1, 1, 0, 0},
  {0, 0, 0, 0, 0}
};

struct Node {
  int x, y;
  int g, h, f;
  Node *parent;
};

// Path array to store steps
Node* path[GRID_ROWS * GRID_COLS];
int pathIndex = 0;

Node* openList[GRID_ROWS * GRID_COLS]; 
bool closedList[GRID_ROWS][GRID_COLS];

// Using Manhattan distance since there are no diagonals
int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Find the node with the lowest f
Node* getLowestFNode() {
  Node* lowest = NULL;
  for (int i = 0; i < GRID_ROWS * GRID_COLS; i++) {
    if (openList[i] != NULL && (lowest == NULL || openList[i]->f < lowest->f)) {
      lowest = openList[i];
    }
  }
  return lowest;
}

// Check if move is valid
bool isValidMove(int x, int y) {
  return (x >= 0 && x < GRID_ROWS && y >= 0 && y < GRID_COLS && maze[x][y] == 0);
}

// A* Algorithm
void aStar(int startX, int startY, int endX, int endY) {
  Node start = {startX, startY, 0, heuristic(startX, startY, endX, endY), 0, NULL};
  start.f = start.g + start.h;
  openList[0] = &start;

  while (true) {
    Node* current = getLowestFNode();
    if (current == NULL) break;

    if (current->x == endX && current->y == endY) {
      while (current != NULL) {
        path[pathIndex++] = current;
        current = current->parent;
      }
      break;
    }

    openList[current->x * GRID_COLS + current->y] = NULL;
    closedList[current->x][current->y] = true;

    int directions[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    for (int i = 0; i < 4; i++) {
      int newX = current->x + directions[i][0];
      int newY = current->y + directions[i][1];

      if (isValidMove(newX, newY) && !closedList[newX][newY]) {
        Node* neighbor = new Node;
        neighbor->x = newX;
        neighbor->y = newY;
        neighbor->g = current->g + 1;
        neighbor->h = heuristic(newX, newY, endX, endY);
        neighbor->f = neighbor->g + neighbor->h;
        neighbor->parent = current;

        openList[newX * GRID_COLS + newY] = neighbor;
      }
    }
  }
}

// Calculate error for PD control
int calculateError() {
  int sensorValues[8];
  int error = 0;
  int totalValue = 0;
  int sumWeighted = 0;

  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(sensorPins[i]);
    totalValue += sensorValues[i];
    sumWeighted += sensorValues[i] * (i - 3);  // Weight the sensors from -3 to +3
  }

  if (totalValue > 0) {
    error = sumWeighted / totalValue;
  }

  return error;
}

// PD Control for motors
void pdControl() {
  int error = calculateError();
  int derivative = error - previousError;
  previousError = error;

  int correction = Kp * error + Kd * derivative;

  int motorSpeedLeft = constrain(baseSpeed - correction, 0, maxSpeed);
  int motorSpeedRight = constrain(baseSpeed + correction, 0, maxSpeed);

  analogWrite(motor1Pin1, motorSpeedLeft);
  analogWrite(motor2Pin1, motorSpeedRight);
}
void moveForwardPD() {
  while (true) {
    pdControl();
    if (/* distance condition using encoders */) {
      break;
    }
  }
}

void turnLeftPD() {
  while (true) {
    analogWrite(motor1Pin1, 0);
    analogWrite(motor2Pin1, baseSpeed);

    pdControl();

    if (/* encoder indicates 90-degree turn */) {
      break;
    }
  }
}

void turnRightPD() {
  while (true) {
    analogWrite(motor1Pin1, baseSpeed);
    analogWrite(motor2Pin1, 0);

    pdControl();

    if (/* encoder indicates 90-degree turn */) {
      break;
    }
  }
}

// Execute path 
void executePath() {
  for (int i = pathIndex - 1; i > 0; i--) {
    int currentX = path[i]->x;
    int currentY = path[i]->y;
    int nextX = path[i-1]->x;
    int nextY = path[i-1]->y;

    if (nextX == currentX && nextY > currentY) {
      moveForwardPD();
    } else if (nextX == currentX && nextY < currentY) {
      turnLeftPD();
      moveForwardPD();
      turnRightPD();
    } else if (nextX > currentX && nextY == currentY) {
      turnRightPD();
      moveForwardPD();
      turnLeftPD();
    } else if (nextX < currentX && nextY == currentY) {
      turnRightPD();
      turnRightPD();
      moveForwardPD();
      turnRightPD();
      turnRightPD();	
    }
  }
}

void setup() {
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);

  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);

  aStar(0, 0, 4, 4);  // Execute A* 

  executePath();  // Follow the computed path
}

void loop() {
  // Loops in case we need restarts
}
