## Odometry simulation:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <Servo.h>
#include <cmath>

// Global variables for odometry
double x = 0.0, y = 0.0, theta = 0.0;
double linear_velocity = 0.0, steering_angle = 0.0;
ros::Time last_time;

// Pin definitions
const int Dir = 7; // Direction pin for DC motor
const int PWM = 6; // PWM pin for DC motor
Servo servo1;

bool autonomous = false; // Autonomous mode flag

// Function to handle cmd_vel messages
void handle_cmd_vel(const geometry_msgs::Twist &cmd_vel) {
    if (!autonomous) {
        // Motor control
        int motorSpeed = int(abs(cmd_vel.linear.x) * 255);
        int motorDirection = (cmd_vel.linear.x >= 0) ? HIGH : LOW;

        digitalWrite(Dir, motorDirection);
        analogWrite(PWM, motorSpeed);

        // Servo control
        int angle;
        if (cmd_vel.linear.x != 0.0) {
            // Map angular.z to servo angle only when linear.x is non-zero
            angle = map(cmd_vel.angular.z, -3.114, 3.114, 20, 90);
        } else {
            // Default servo position (straight) when car is stationary
            angle = 55; // Midpoint for your range
        }
        servo1.write(angle);

        // Update global velocities
        linear_velocity = cmd_vel.linear.x;
        steering_angle = cmd_vel.angular.z;
    }
}

// Function to update odometry
void updateOdometry(double dt) {
    if (linear_velocity == 0.0) {
        // Car is stationary; no changes in x, y, or theta
        return;
    }

    // Ackermann kinematics: compute delta_x, delta_y, and delta_theta
    double delta_x = linear_velocity * std::cos(theta) * dt;
    double delta_y = linear_velocity * std::sin(theta) * dt;
    double delta_theta = 0.0;

    if (std::abs(steering_angle) > 1e-6) {
        double turning_radius = linear_velocity / std::tan(steering_angle);
        delta_theta = linear_velocity * dt / turning_radius;
    }

    x += delta_x;
    y += delta_y;
    theta += delta_theta;

    // Normalize theta to keep it within [-pi, pi]
    theta = std::atan2(std::sin(theta), std::cos(theta));
}

// Function to publish odometry
void publishOdometry(ros::Publisher &odom_pub, ros::Time current_time) {
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    // Position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.z = std::sin(theta / 2.0);
    odom.pose.pose.orientation.w = std::cos(theta / 2.0);

    // Twist
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = linear_velocity;
    odom.twist.twist.angular.z = steering_angle;

    odom_pub.publish(odom);
}

// Function to broadcast transform
void publishTransform(tf::TransformBroadcaster &tf_broadcaster, ros::Time current_time) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x;
    transform.transform.translation.y = y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.z = std::sin(theta / 2.0);
    transform.transform.rotation.w = std::cos(theta / 2.0);

    // Normalize quaternion
    double norm = std::sqrt(std::pow(transform.transform.rotation.w, 2) +
                            std::pow(transform.transform.rotation.z, 2));
    transform.transform.rotation.w /= norm;
    transform.transform.rotation.z /= norm;

    tf_broadcaster.sendTransform(transform);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rc_car_control");
    ros::NodeHandle nh;

    // Publishers and subscribers
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 10, handle_cmd_vel);

    // TF broadcaster
    tf::TransformBroadcaster tf_broadcaster;

    // Servo setup
    servo1.attach(9); // Attach servo to pin 9

    // Initialize pins
    pinMode(Dir, OUTPUT);
    pinMode(PWM, OUTPUT);

    last_time = ros::Time::now();
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        updateOdometry(dt);
        publishOdometry(odom_pub, current_time);
        publishTransform(tf_broadcaster, current_time);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

```
