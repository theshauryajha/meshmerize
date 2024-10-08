const int sensorCount = 8;
int sensorValues[sensorCount];

void setup(){
  // put your setup code here, to run once:
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);

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
  sensorValues[0] = analogRead(A0);
  sensorValues[1] = analogRead(A1);
  sensorValues[2] = analogRead(A2);
  sensorValues[3] = analogRead(A3);
  sensorValues[4] = analogRead(A4);
  sensorValues[5] = analogRead(A5);
  sensorValues[6] = analogRead(A6);
  sensorValues[7] = analogRead(A7);  
}
