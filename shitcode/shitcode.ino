int speedPin = 5;
int dir1 = 4;
int dir2 = 3;
bool dir = 0;
int mSpeed = 90;

void setup() {
  // put your setup code here, to run once:
  pinMode(speedPin, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);

  Serial.begin(9600);
  Serial.println("Please enter the motor speed: ");
  while(Serial.available()==0){}
  mSpeed = Serial.read();
  Serial.println(String(mSpeed));

  Serial.println("Please enter the direction: ");
  while(Serial.available()==0){}
  dir = Serial.read();
  Serial.println(String(dir));
}

void loop() {
  // put your main code here, to run repeatedly:
  

  digitalWrite(dir1, dir);
  digitalWrite(dir2, !dir);
  analogWrite(speedPin, 255);
  delay(25);
  analogWrite(speedPin, mSpeed);
  delay(5000);
}