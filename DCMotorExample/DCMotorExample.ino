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

  Serial.begin(9600);//Starts the serial comms

  //Look for user input on motor speed
  Serial.println("Please enter the motor speed:");
  while(Serial.available()==0){}
  mSpeed = Serial.parseInt();
  Serial.println("Speed is set to: " + String(mSpeed));
  delay(25);

  Serial.end();//Ends the serial comms
  Serial.begin(9600);//Restarts the serial
  
  //Look for user input on direction
  Serial.println("Please enter the direction: ");
  while(Serial.available()==0){}
  dir = Serial.read();
  Serial.println(String(dir));
  
  Serial.end();//Ends the serial comms
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