const int sensorCount = 8;
int sensorValues[sensorCount];

void setup(){
  // put your setup code here, to run once:
  pinMode(A8,INPUT);
  pinMode(A9,INPUT);
  pinMode(A10,INPUT);
  pinMode(A11,INPUT);
  pinMode(A12,INPUT);
  pinMode(A13,INPUT);
  pinMode(A14,INPUT);
  pinMode(A15,INPUT);

  Serial.begin(9600);
}

void loop(){
  // put your main code here, to run repeatedly:
  read();
  for (int i = 0; i < 8; i++){
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();

  delay(100);
}

void read(){
  sensorValues[0] = analogRead(A8);
  sensorValues[1] = analogRead(A9);
  sensorValues[2] = analogRead(A10);
  sensorValues[3] = analogRead(A11);
  sensorValues[4] = analogRead(A12);
  sensorValues[5] = analogRead(A13);
  sensorValues[6] = analogRead(A14);
  sensorValues[7] = analogRead(A15);  
}
