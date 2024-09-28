// define motor pins
#define leftMotor1 2
#define leftMotor2 3
#define leftMotorPWM 6
#define rightMotor1 4
#define rightMotor2 5
#define rightMotorPWM 7
#define STDBY 8

// define sensor pins
#define leftMostSensor 15
#define leftCenterSensor 14
#define centerSensor 13
#define rightCenterSensor 12
#define rightMostSensor 11

// threshold to detect white line
#define th 100

// speed control variables
#define baseSpeed 120

// variables for sensor values
int l, lc, c, rc, r;

void setup() {
    // setup sensor pins as input
    pinMode(leftMostSensor, INPUT);
    pinMode(leftCenterSensor, INPUT);
    pinMode(centerSensor, INPUT);
    pinMode(rightCenterSensor, INPUT);
    pinMode(rightMostSensor, INPUT);

    // setup motor pins as output
    pinMode(leftMotor1 ,OUTPUT);
    pinMode(leftMotor2 ,OUTPUT);
    pinMode(leftMotorPWM ,OUTPUT);
    pinMode(rightMotor1 ,OUTPUT);
    pinMode(rightMotor2 ,OUTPUT);
    pinMode(rightMotorPWM ,OUTPUT);

    // setup standby pin, initally high, low to kill motors
    pinMode(STDBY ,OUTPUT);
    digitalWrite(STDBY, HIGH);
}

void loop() {
    read();
    
    if (l > th && lc < th && c < th && rc < th && r > th){
      digitalWrite(STDBY, HIGH);
      driveMotors(baseSpeed, baseSpeed);
    }
    else
      digitalWrite(STDBY, LOW);
    
}

void read() {
    l = analogRead(leftMostSensor);
    lc = analogRead(leftCenterSensor);
    c = analogRead(centerSensor);
    rc = analogRead(rightCenterSensor);
    r = analogRead(rightMostSensor);
}

void driveMotors(int left, int right) {
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);

    analogWrite(leftMotorPWM, left);
    analogWrite(rightMotorPWM, right);
}
