#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/time.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/transform_stamped.h>

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 accel;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

rcl_publisher_t imuPublisher;
sensor_msgs__msg__Imu imuMsg;

rcl_publisher_t transformPublisher;
geometry_msgs__msg__TransformStamped transformMsg;

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void imu__update() {

  double roll = 0.0;   // Replace with actual roll value in radians
  double pitch = 0.0;  // Replace with actual pitch value in radians
  double yaw = 0.0;    // Replace with actual yaw value in radians

  int16_t ax, ay, az;
  accel.getAcceleration(&ax, &ay, &az);

  double accel_scale = 9.81 / 16384.0;
  double gyro_scale = (250.0 / 32768.0) * (3.14 / 180.0);

  // Read gyroscope data
  int16_t gx, gy, gz;
  accel.getRotation(&gx, &gy, &gz);

  // Convert raw accelerometer values to acceleration in m/s^2
  double accelX = ax * accel_scale;
  double accelY = ay * accel_scale;
  double accelZ = az * accel_scale;

  // Convert raw gyroscope values to angular velocity in radians per second
  double gyroX = gx * gyro_scale;
  double gyroY = gy * gyro_scale;
  double gyroZ = gz * gyro_scale;

  // Complementary filter to estimate roll and pitch angles
  double dt = 0.1;      // Adjust this value based on your loop rate
  double alpha = 0.98;  // Complementary filter coefficient

  double acc_roll = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ));
  double acc_pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ));

  double gyro_roll = acc_roll + gyroX * dt;
  double gyro_pitch = acc_pitch + gyroY * dt;

  roll = alpha * gyro_roll + (1 - alpha) * acc_roll;
  pitch = alpha * gyro_pitch + (1 - alpha) * acc_pitch;

  // Calculate quaternion values
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  imuMsg.orientation.w = cr * cp * cy + sr * sp * sy;
  imuMsg.orientation.x = sr * cp * cy - cr * sp * sy;
  imuMsg.orientation.y = cr * sp * cy + sr * cp * sy;
  imuMsg.orientation.z = cr * cp * sy - sr * sp * cy;

  double accel_cov = 0.00001;

  imuMsg.linear_acceleration.x = accelX;
  imuMsg.linear_acceleration.y = accelY;
  imuMsg.linear_acceleration.z = accelZ;
  imuMsg.linear_acceleration_covariance[0] = accel_cov;
  imuMsg.linear_acceleration_covariance[4] = accel_cov;
  imuMsg.linear_acceleration_covariance[8] = accel_cov;

  imuMsg.orientation.w = cr * cp * cy + sr * sp * sy;
  imuMsg.orientation.x = sr * cp * cy - cr * sp * sy;
  imuMsg.orientation.y = cr * sp * cy + sr * cp * sy;
  imuMsg.orientation.z = cr * cp * sy - sr * sp * cy;

  double orientation_cov = 0.00001;
  imuMsg.orientation_covariance[0] = orientation_cov;
  imuMsg.orientation_covariance[4] = orientation_cov;
  imuMsg.orientation_covariance[8] = orientation_cov;

  // Tranform message
  uint32_t currentMillis = millis();
  transformMsg.header.stamp.sec = currentMillis / 1000;
  transformMsg.header.stamp.nanosec = (currentMillis % 1000) * 1e6;
  transformMsg.header.frame_id.data = (char *)"imu_link";
  transformMsg.child_frame_id.data = (char *)"base_link";
  transformMsg.transform.translation.x = 0.0;
  transformMsg.transform.translation.y = 0.0;
  transformMsg.transform.translation.z = 0.0;
  transformMsg.transform.rotation = imuMsg.orientation;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    //RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    imu__update();
    rcl_publish(&imuPublisher, &imuMsg, NULL);
    rcl_publish(&transformPublisher, &transformMsg, NULL);
  }
}

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Wire.begin();
  accel.initialize();

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create TF publisher
  RCCHECK(rclc_publisher_init_default(
    &transformPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TransformStamped),
    "tf_msg"));

  // create IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imuPublisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu_msg"));

  // create timer,
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  imuMsg.header.frame_id.data = (char *)"imu_link";
  transformMsg.header.frame_id.data = (char *)"imu_link";
  transformMsg.child_frame_id.data = (char *)"base_link";

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
