```
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

// HC-SR04 Front Right Sensor
#define echoPin1 8 // attach pin D12 Arduino to pin Echo of HC-SR04
#define trigPin1 9 //attach pin D11 PWM Arduino to pin Trig of HC-SR04

// defines variables
long durationFR; // variable for the duration of sound wave travel
int SoundDistanceFR; // variable for the distance measurement

// HC-SR04 Front Left Sensor
#define echoPin2 7 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin2 6 //attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationFL; // variable for the duration of sound wave travel
int SoundDistanceFL; // variable for the distance measurement

// HC-SR04 Rear Right Sensor
#define echoPin3 2 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin3 5 //attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationRR; // variable for the duration of sound wave travel
int SoundDistanceRR; // variable for the distance measurement

// HC-SR04 Rear Left Sensor
#define echoPin4 12 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin4 11 //attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationRL; // variable for the duration of sound wave travel
int SoundDistanceRL; // variable for the distance measurement

VL53L0X sensor;

int Dir = 4; // Yellow
int PWM = 3; // White

int pos = 0;    // variable to store the servo position
Servo servo1; // Pin 10


void setup() {
  // put your setup code here, to run once:

  //Motor

  pinMode(PWM, OUTPUT);
  pinMode(Dir, OUTPUT);

  // Servo

  servo1.attach(10);
  servo1.write(70); // turn a little bit right
  delay(500);
  servo1.write(56); // wheels back to the center
  delay(500);

  // Lazer Sensor

  Serial.begin(9600); // Serial Communication is starting with 9600 of baudrate speed
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.startContinuous();

  // HC-SR04 sensors

  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT

  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT

  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin3, INPUT); // Sets the echoPin as an INPUT

  pinMode(trigPin4, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(echoPin4, INPUT); // Sets the echoPin as an INPUT

}

void loop() {

  // Lazer Sensor

  int dist = 0;
  dist = sensor.readRangeContinuousMillimeters();
  dist = dist - 30; // -30 is to compensate for error. Change or set it to zero to make it work for your sensor
  Serial.print("Lazer Front distance: ");
  Serial.print(dist);
  Serial.print("mm");
  Serial.println();

  //Motor

  if (dist <= 400) {

    servo1.write(56);
    analogWrite(PWM, 0);
    delay(500);

    if (SoundSensorFL() >= SoundSensorFR()) {
      Serial.print("SoundSensorRR() >= SoundSensorRL()");
      Serial.println();
      analogWrite(PWM, 0);
      servo1.write(80);
      if (SoundSensorRR() >= 20) {
        digitalWrite(Dir, LOW);
        analogWrite(PWM, 57);
        delay(500);
        analogWrite(PWM, 0);
        servo1.write(30);
        digitalWrite(Dir, HIGH);
        analogWrite(PWM, 57);
        delay(500);
      }
      else {
        servo1.write(80);
        delay(500);
        servo1.write(30);
        delay(500);
      }
    }
    else if (SoundSensorFR() >= SoundSensorFL()) {
      Serial.print("SoundSensorRL() >= SoundSensorRR()");
      Serial.println();
      analogWrite(PWM, 0);
      servo1.write(30);
      if (SoundSensorRL() >= 20) {
        digitalWrite(Dir, LOW);
        analogWrite(PWM, 57);
        delay(500);
        analogWrite(PWM, 0);
        servo1.write(80);
        digitalWrite(Dir, HIGH);
        analogWrite(PWM, 57);
        delay(500);
      }
      else {
        servo1.write(30);
        delay(500);
        servo1.write(80);
        delay(500);
      }
    }
  }

  else {

    servo1.write(56);
    digitalWrite(Dir, HIGH);
    analogWrite(PWM, 57);

    while (SoundSensorFL() >= 10 && SoundSensorFL() <= 35) {
      servo1.write(80);
      digitalWrite(Dir, HIGH);
      analogWrite(PWM, 57);
    }

    while (SoundSensorFR() >= 10 && SoundSensorFR() <= 35) {
      servo1.write(30);
      digitalWrite(Dir, HIGH);
      analogWrite(PWM, 57);
    }

    if (SoundSensorFR() <= 10) {
      digitalWrite(Dir, LOW);
      analogWrite(PWM, 0);
      servo1.write(80);
      delay(500);
      digitalWrite(Dir, LOW);
      analogWrite(PWM, 57);
      delay(500);
      servo1.write(30);
      digitalWrite(Dir, HIGH);
      analogWrite(PWM, 57);
      delay(500);
    }

    if (SoundSensorFL() <= 10) {
      digitalWrite(Dir, LOW);
      analogWrite(PWM, 0);
      servo1.write(30);
      delay(500);
      digitalWrite(Dir, LOW);
      analogWrite(PWM, 57);
      delay(500);
      servo1.write(80);
      digitalWrite(Dir, HIGH);
      analogWrite(PWM, 57);
      delay(500);
    }
  }
}

// Functions :

int SoundSensorFR() {

  // HC-SR04 Front Right Sensor

  // Clears the trigPin condition
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationFR = pulseIn(echoPin1, HIGH);
  // Calculating the distance
  SoundDistanceFR = durationFR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Front Right SoundDistance: ");
  Serial.print(SoundDistanceFR);
  Serial.println(" cm");
  return SoundDistanceFR;
}

int SoundSensorFL() {

  // HC-SR04 Front Left Sensor

  // Clears the trigPin condition
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationFL = pulseIn(echoPin2, HIGH);
  // Calculating the distance
  SoundDistanceFL = durationFL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Front Left SoundDistance: ");
  Serial.print(SoundDistanceFL);
  Serial.println(" cm");
  return SoundDistanceFL;
}

int SoundSensorRR() {

  // HC-SR04 Rear Right Sensor

  // Clears the trigPin condition
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationRR = pulseIn(echoPin3, HIGH);
  // Calculating the distance
  SoundDistanceRR = durationRR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Rear Right SoundDistance: ");
  Serial.print(SoundDistanceRR);
  Serial.println(" cm");
  return SoundDistanceRR;
}

int SoundSensorRL() {

  // HC-SR04 Rear Left Sensor

  // Clears the trigPin condition
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  durationRL = pulseIn(echoPin4, HIGH);
  // Calculating the distance
  SoundDistanceRL = durationRL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Rear Left SoundDistance: ");
  Serial.print(SoundDistanceRL);
  Serial.println(" cm");
  return SoundDistanceRL;
}
```
