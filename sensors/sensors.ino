#define O1 A0
#define O2 A1
#define O3 A2
#define O4 A3
#define O5 A4

void setup() {
  // put your setup code here, to run once:
  pinMode(O1, INPUT);
  pinMode(O2, INPUT);
  pinMode(O3, INPUT);
  pinMode(O4, INPUT);
  pinMode(O5, INPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:

  int sensor1 = analogRead(O1);
  int sensor2 = analogRead(O2);
  int sensor3 = analogRead(O3);
  int sensor4 = analogRead(O4);
  int sensor5 = analogRead(O5);

  int sensorArray[5] = {sensor1, sensor2, sensor3, sensor4, sensor5};
  
  Serial.println(sensorArray[4]);
}
