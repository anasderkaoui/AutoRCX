## Odometry simulation:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>

class FakeOdomAckermann {
public:
    FakeOdomAckermann() {
        // Initialize ROS node
        cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &FakeOdomAckermann::cmdVelCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        last_time = ros::Time::now();
    }

    void spin() {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            ros::Time current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            last_time = current_time;

            // Apply Ackermann kinematic equations
            double delta_x = linear_velocity * std::cos(theta) * dt;
            double delta_y = linear_velocity * std::sin(theta) * dt;
            double delta_theta = 0.0;

            if (steering_angle != 0.0) {
                double turning_radius = linear_velocity / std::tan(steering_angle);
                delta_theta = linear_velocity * dt / turning_radius;
            }

            x += delta_x;
            y += delta_y;
            theta += delta_theta;

            // Publish odometry message
            publishOdometry(current_time);

            // Publish TF transform
            publishTransform(current_time);

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    double x, y, theta; // Robot's pose
    double linear_velocity = 0.0;
    double steering_angle = 0.0;

    ros::Time last_time;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        linear_velocity = msg->linear.x;  // Velocity of the car
        steering_angle = msg->angular.z; // Steering angle in radians
    }

    void publishOdometry(const ros::Time& current_time) {
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Pose
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.z = std::sin(theta / 2.0);
        odom.pose.pose.orientation.w = std::cos(theta / 2.0);

        // Velocity
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.angular.z = linear_velocity * std::tan(steering_angle);

        odom_pub.publish(odom);
    }

    void publishTransform(const ros::Time& current_time) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = current_time;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";

        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.z = std::sin(theta / 2.0);
        transform.transform.rotation.w = std::cos(theta / 2.0);

        tf_broadcaster.sendTransform(transform);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_odom_ackermann");
    FakeOdomAckermann fake_odom;
    fake_odom.spin();
    return 0;
}
```

## Arduino code to control the car:

```c
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
  int angle = map(cmd_vel.angular.z, -1, 1, 20, 90); // Maximum left is "20" degrees corresponds to -1 in ros, maximum right is "90" degrees corresponds to 1 in ros
  //int centered_angle = angle - 90 + 56;
  //servo1.write(constrain(centered_angle, 0, 180));
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
```
## Corresponding pub/sub to control the car with a wireless controller:

```cpp
// This code works great in order to control the car wirelessly with a ps4 controller. The left joystick is for steering and the right joystick is for speed control.

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

class CarControl {
public:
    CarControl() : nh_(), autonomous_(false) {
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        toggle_pub_ = nh_.advertise<std_msgs::Bool>("toggle_autonomy", 1);
        joy_sub_ = nh_.subscribe("joy", 10, &CarControl::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        geometry_msgs::Twist twist;
        std_msgs::Bool toggle_msg;

        // Toggle autonomous mode with Square button
        if (joy->buttons[0] == 1) { // Square button
            autonomous_ = !autonomous_;
            toggle_msg.data = autonomous_;
            toggle_pub_.publish(toggle_msg);
            ros::Duration(0.5).sleep(); // Debounce delay
        }

        // Control the car manually if not in autonomous mode
        if (!autonomous_) {
            // Map the R2 and L2 triggers for forward and backward motion
            //double forward = (1.0 - joy->axes[3]) / 2.0; // Scale from 0.0 to 1.0
            //double backward = (1.0 - joy->axes[4]) / 2.0; // Scale from 0.0 to 1.0
            //double motion = 
            
	    // Map the right joystick for speed
            twist.linear.x = joy->axes[5] * 0.1; //adjust speed as needed for forward or backward motion

            // Map the left joystick for steering
            twist.angular.z = joy->axes[0]; // Left joystick horizontal

            // Publish the twist message to control the car
            cmd_pub_.publish(twist);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_pub_;
    ros::Publisher toggle_pub_;
    ros::Subscriber joy_sub_;
    bool autonomous_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "car_control_node");
    CarControl car_control;

    ros::spin();
    return 0;
}

```
