//declare where the trig and echo pins are located
const int trigPin = 11;
const int echoPin = 12;

//declare where the led pins are located
const int greenLEDPin = 4;
const int yellowLEDPin = 3;
const int redLEDPin = 2;

//declare int for use with distance
double prevDistance = 0;


void setup() {
  //initialize trig, echo, and LED pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(yellowLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);

  //begin serial monitor
  Serial.begin(9600);
}

void loop() {

  //Set all LED pins to 0
  analogWrite(greenLEDPin, 0);
  analogWrite(yellowLEDPin, 0);
  analogWrite(redLEDPin, 0);

  //Set trig to LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  //Set trig to HIGH to send out sonic burst
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);

  //Set trig to LOW to stop sonic burst
  digitalWrite(trigPin, LOW);

  //print the distance to the serial monitor
  Serial.println("Distance: " + String(findDistance()) + " cm");
  delay(100);

  /*
  //change LED based on distance;
  changeLED(findDistance());
  */
}

double findDistance(){
  /*
  use the pulseIn function to find the 
  time (in microseconds) echoPin is HIGH
  */
  double duration = pulseIn(echoPin, HIGH);

  /*
  calculate distance by multiplying 
  the duration by the speed of sound 
  in cm per second (0.0343) and divide
  by 2 since it is time for sonic burst
  to reach to an object and back
  */
  double distance = (duration * .0343) / 2;
  
  //return the distance
  return distance;
}


/*
A method to change the led state by checking the distance
distance < 45.75 -> Green
distance < 91.44 -> Yellow
distance < 137.16 -> Red
*/
/*
void changeLED(double distance){
  if(prevDistance != distance){
    if(distance <= 45.75){
      analogWrite(greenLEDPin, 255);
      analogWrite(yellowLEDPin, 0);
      analogWrite(redLEDPin, 0);
    }else if(distance <= 91.44){
      analogWrite(greenLEDPin, 0);
      analogWrite(yellowLEDPin, 255);
      analogWrite(redLEDPin, 0);
    }else if(distance > 91.44){
      analogWrite(greenLEDPin, 0);
      analogWrite(yellowLEDPin, 0);
      analogWrite(redLEDPin, 255);
    }
  }
  prevDistance = distance;
}
*/