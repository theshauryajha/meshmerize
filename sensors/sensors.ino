#define sensorCount 4
const int sensorPins[sensorCount] = {15, 2, 4, 16};
int sensorValues[sensorCount];

void setup(){
  // put your setup code here, to run once:
  for (int i = 0; i < sensorCount; i++){
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  read();

  for (int i = 0; i < sensorCount; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}

void read(){
  for (int i = 0; i < sensorCount; i++){
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
}