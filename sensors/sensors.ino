#define LEFT 15
#define LEFT_CENTER 14
#define CENTER 13
#define RIGHT_CENTER 12
#define RIGHT 11

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT, INPUT);
  pinMode(LEFT_CENTER, INPUT);
  pinMode(CENTER, INPUT);
  pinMode(RIGHT_CENTER, INPUT);
  pinMode(RIGHT, INPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  int sensor1 = analogRead(LEFT);
  int sensor2 = analogRead(LEFT_CENTER);
  int sensor3 = analogRead(CENTER);
  int sensor4 = analogRead(RIGHT_CENTER);
  int sensor5 = analogRead(RIGHT);

  int sensorValues[5] = {sensor1, sensor2, sensor3, sensor4, sensor5};

  for (int i = 0; i < 5; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}
