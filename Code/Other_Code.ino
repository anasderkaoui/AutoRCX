#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

// HC-SR04 Front Right Sensor
#define echoPin1 8 // attach pin D12 Arduino to pin Echo of HC-SR04
#define trigPin1 9 // attach pin D11 PWM Arduino to pin Trig of HC-SR04

// defines variables
long durationFR; // variable for the duration of sound wave travel
int SoundDistanceFR; // variable for the distance measurement

// HC-SR04 Front Left Sensor
#define echoPin2 7 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin2 6 // attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationFL; // variable for the duration of sound wave travel
int SoundDistanceFL; // variable for the distance measurement

// HC-SR04 Rear Right Sensor
#define echoPin3 2 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin3 5 // attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationRR; // variable for the duration of sound wave travel
int SoundDistanceRR; // variable for the distance measurement

// HC-SR04 Rear Left Sensor
#define echoPin4 12 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin4 11 // attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationRL; // variable for the duration of sound wave travel
int SoundDistanceRL; // variable for the distance measurement

VL53L0X sensor;

int Dir = 4; // Yellow
int PWM = 3; // White

int pos = 0;    // variable to store the servo position
Servo servo1; // Pin 10

ros::NodeHandle nh;

void handle_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
  // Motor control
  int motorSpeed = int(abs(cmd_vel.linear.x) * 255);
  int motorDirection = (cmd_vel.linear.x >= 0) ? HIGH : LOW;

  digitalWrite(Dir, motorDirection);
  analogWrite(PWM, motorSpeed);

  // Servo control
  int angle = map(cmd_vel.angular.z, -1, 1, 20, 90); // Left = 20 == -1 in ros, Right = 90 == 1 in ros
  servo1.write(angle);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", handle_cmd_vel);

void setup() {
  // Motor
  pinMode(PWM, OUTPUT);
  pinMode(Dir, OUTPUT);

  // Servo
  servo1.attach(10);
  servo1.write(56); // wheels back to the center

  // Lidar sensor
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

  // Initialize ROS node handle
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // Lidar sensor
  int dist = sensor.readRangeContinuousMillimeters() - 30; // -30 is to compensate for error
  Serial.print("Lazer Front distance: ");
  Serial.print(dist);
  Serial.print("mm");
  Serial.println();

  // Call sound sensor functions and use their values if necessary
  int distanceFR = SoundSensorFR();
  int distanceFL = SoundSensorFL();
  int distanceRR = SoundSensorRR();
  int distanceRL = SoundSensorRL();

  nh.spinOnce();
  delay(10);
}

// Functions :

int SoundSensorFR() {
  // HC-SR04 Front Right Sensor
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  durationFR = pulseIn(echoPin1, HIGH);
  SoundDistanceFR = durationFR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Front Right SoundDistance: ");
  Serial.print(SoundDistanceFR);
  Serial.println(" cm");
  return SoundDistanceFR;
}

int SoundSensorFL() {
  // HC-SR04 Front Left Sensor
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  durationFL = pulseIn(echoPin2, HIGH);
  SoundDistanceFL = durationFL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Front Left SoundDistance: ");
  Serial.print(SoundDistanceFL);
  Serial.println(" cm");
  return SoundDistanceFL;
}

int SoundSensorRR() {
  // HC-SR04 Rear Right Sensor
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  durationRR = pulseIn(echoPin3, HIGH);
  SoundDistanceRR = durationRR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Rear Right SoundDistance: ");
  Serial.print(SoundDistanceRR);
  Serial.println(" cm");
  return SoundDistanceRR;
}

int SoundSensorRL() {
  // HC-SR04 Rear Left Sensor
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  durationRL = pulseIn(echoPin4, HIGH);
  SoundDistanceRL = durationRL * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  Serial.print("Rear Left SoundDistance: ");
  Serial.print(SoundDistanceRL);
  Serial.println(" cm");
  return SoundDistanceRL;
}
