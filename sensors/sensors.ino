 0int sensorCount = 8;
int sensorValues[sensorCount];

void setup(){
  // put your setup code here, to run once:
  for (int i = 11; i <= 18; i++){
    pinMode(i, INPUT);
  }

  Serial.begin(9600);
}

void loop(){
  // put ypour main code here to run repeatedly:
  read();
  for (int i = 0; i < 8; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}

void read(){
  for (int i = 0; i < 8; i++){
    sensorValues[i] = analogRead(i+11);
  }
}