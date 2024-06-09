#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>

// HC-SR04 Front Right Sensor
#define echoPin1 8 // attach pin D12 Arduino to pin Echo of HC-SR04
#define trigPin1 9 // attach pin D11 PWM Arduino to pin Trig of HC-SR04

// HC-SR04 Front Left Sensor
#define echoPin2 7 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin2 6 // attach pin D6 Arduino to pin Trig of HC-SR04

// HC-SR04 Rear Right Sensor
#define echoPin3 2 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin3 5 // attach pin D6 Arduino to pin Trig of HC-SR04

// HC-SR04 Rear Left Sensor
#define echoPin4 12 // attach pin D7 Arduino to pin Echo of HC-SR04
#define trigPin4 11 // attach pin D6 Arduino to pin Trig of HC-SR04

// defines variables
long durationFR; // variable for the duration of sound wave travel
int SoundDistanceFR; // variable for the distance measurement

long durationFL; // variable for the duration of sound wave travel
int SoundDistanceFL; // variable for the distance measurement

long durationRR; // variable for the duration of sound wave travel
int SoundDistanceRR; // variable for the distance measurement

long durationRL; // variable for the duration of sound wave travel
int SoundDistanceRL; // variable for the distance measurement

VL53L0X sensor;

int Dir = 4; // Yellow
int PWM = 3; // White

int pos = 0;    // variable to store the servo position
Servo servo1; // Pin 10

ros::NodeHandle nh;

// Publisher for ultrasonic sensor data
sensor_msgs::Range front_right_msg;
ros::Publisher front_right_pub("front_right_range", &front_right_msg);

sensor_msgs::Range front_left_msg;
ros::Publisher front_left_pub("front_left_range", &front_left_msg);

sensor_msgs::Range rear_right_msg;
ros::Publisher rear_right_pub("rear_right_range", &rear_right_msg);

sensor_msgs::Range rear_left_msg;
ros::Publisher rear_left_pub("rear_left_range", &rear_left_msg);

// Publisher for lidar data
sensor_msgs::LaserScan laser_msg;
ros::Publisher laser_pub("laser_scan", &laser_msg);

void handle_cmd_vel(const geometry_msgs::Twist& cmd_vel) {
  // Motor control
  int motorSpeed = int(abs(cmd_vel.linear.x) * 255);
  int motorDirection = (cmd_vel.linear.x >= 0) ? HIGH : LOW;

  // Obstacle avoidance logic
  if (SoundSensorFR() < 20 || SoundSensorFL() < 20 || SoundSensorRR() < 20 || SoundSensorRL() < 20) {
    // Stop the car if an obstacle is detected
    motorSpeed = 0;
  }

  digitalWrite(Dir, motorDirection);
  analogWrite(PWM, motorSpeed);

  // Servo control
  int center = 56;  // Your center position
  int right_turn = 95;  // Your right turn position
  int left_turn = 56 - (right_turn - 56); // Calculate the corresponding left turn

  int angle = map(cmd_vel.angular.z, -1, 1, left_turn, right_turn);
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

  // Initialize Range messages
  front_right_msg.header.frame_id = "front_right_sensor";
  front_right_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  front_right_msg.field_of_view = 0.0523; // example value
  front_right_msg.min_range = 0.02;
  front_right_msg.max_range = 4.0;

  front_left_msg.header.frame_id = "front_left_sensor";
  front_left_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  front_left_msg.field_of_view = 0.0523; // example value
  front_left_msg.min_range = 0.02;
  front_left_msg.max_range = 4.0;

  rear_right_msg.header.frame_id = "rear_right_sensor";
  rear_right_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rear_right_msg.field_of_view = 0.0523; // example value
  rear_right_msg.min_range = 0.02;
  rear_right_msg.max_range = 4.0;

  rear_left_msg.header.frame_id = "rear_left_sensor";
  rear_left_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  rear_left_msg.field_of_view = 0.0523; // example value
  rear_left_msg.min_range = 0.02;
  rear_left_msg.max_range = 4.0;

  // Initialize LaserScan message
  laser_msg.header.frame_id = "laser_frame";
  laser_msg.angle_min = -1.5708; // -90 degrees
  laser_msg.angle_max = 1.5708;  // 90 degrees
  laser_msg.angle_increment = 0.0174533; // 1 degree
  laser_msg.time_increment = 0.0;
  laser_msg.scan_time = 0.1;
  laser_msg.range_min = 0.02;
  laser_msg.range_max = 4.0;

  // Advertise ROS publishers
  nh.advertise(front_right_pub);
  nh.advertise(front_left_pub);
  nh.advertise(rear_right_pub);
  nh.advertise(rear_left_pub);
  nh.advertise(laser_pub);
}

void loop() {
  // Get sensor data
  int distanceFR = SoundSensorFR();
  int distanceFL = SoundSensorFL();
  int distanceRR = SoundSensorRR();
  int distanceRL = SoundSensorRL();
  int dist = sensor.readRangeContinuousMillimeters() - 30; // Lidar distance

  // Update and publish ultrasonic sensor data
  front_right_msg.range = distanceFR / 100.0; // convert to meters
  front_right_pub.publish(&front_right_msg);

  front_left_msg.range = distanceFL / 100.0; // convert to meters
  front_left_pub.publish(&front_left_msg);

  rear_right_msg.range = distanceRR / 100.0; // convert to meters
  rear_right_pub.publish(&rear_right_msg);

  rear_left_msg.range = distanceRL / 100.0; // convert to meters
  rear_left_pub.publish(&rear_left_msg);

  // Update and publish lidar data
  laser_msg.header.stamp = nh.now();
  laser_msg.ranges.resize(1);
  laser_msg.ranges[0] = dist / 1000.0; // convert to meters
  laser_pub.publish(&laser_msg);

  nh.spinOnce();
  delay(100); // Adjust the delay as needed
}

// Functions for ultrasonic sensors
int SoundSensorFR() {
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  durationFR = pulseIn(echoPin1, HIGH);
  SoundDistanceFR = durationFR * 0.034 / 2;
  return SoundDistanceFR;
}

int SoundSensorFL() {
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  durationFL = pulseIn(echoPin2, HIGH);
  SoundDistanceFL = durationFL * 0.034 / 2;
  return SoundDistanceFL;
}

int SoundSensorRR() {
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin3, LOW);
  durationRR = pulseIn(echoPin3, HIGH);
  SoundDistanceRR = durationRR * 0.034 / 2;
  return SoundDistanceRR;
}

int SoundSensorRL() {
  digitalWrite(trigPin4, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin4, LOW);
  durationRL = pulseIn(echoPin4, HIGH);
  SoundDistanceRL = durationRL * 0.034 / 2;
  return SoundDistanceRL;
}
