#define leftMostSensor 15
#define leftCenterSensor 14
#define centerSensor 13
#define rightCenterSensor 12
#define rightMostSensor 11

void setup(){
  // put your setup code here, to run once:
  pinMode(leftMostSensor, INPUT);
  pinMode(leftCenterSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightCenterSensor, INPUT);
  pinMode(rightMostSensor, INPUT);

  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  int sensor1 = analogRead(leftMostSensor);
  int sensor2 = analogRead(leftCenterSensor);
  int sensor3 = analogRead(centerSensor);
  int sensor4 = analogRead(rightCenterSensor);
  int sensor5 = analogRead(rightMostSensor);

  int sensorValues[5] ={sensor1, sensor2, sensor3, sensor4, sensor5};

  for (int i = 0; i < 5; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}
