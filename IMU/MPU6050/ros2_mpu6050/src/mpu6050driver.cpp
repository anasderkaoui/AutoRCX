#include "mpu6050driver/mpu6050driver.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

MPU6050Driver::MPU6050Driver()
    : Node("mpu6050publisher"), mpu6050_{std::make_unique<MPU6050Sensor>()}
{
  // Declare parameters
  declareParameters();
  // Set parameters
  mpu6050_->setGyroscopeRange(
      static_cast<MPU6050Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
  mpu6050_->setAccelerometerRange(
      static_cast<MPU6050Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
  mpu6050_->setDlpfBandwidth(
      static_cast<MPU6050Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
  mpu6050_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                               this->get_parameter("gyro_y_offset").as_double(),
                               this->get_parameter("gyro_z_offset").as_double());
  mpu6050_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                   this->get_parameter("accel_y_offset").as_double(),
                                   this->get_parameter("accel_z_offset").as_double());
  // Check if we want to calibrate the sensor
  if (this->get_parameter("calibrate").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Calibrating...");
    mpu6050_->calibrate();
  }
  mpu6050_->printConfig();
  mpu6050_->printOffsets();
  
  // Create a TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  // Your main loop
  timer_ = create_wall_timer(std::chrono::milliseconds(10), [this]() {      
  // Create publisher
  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_link", 50);
  std::chrono::duration<int64_t, std::milli> frequency =
      1000ms / this->get_parameter("gyro_range").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&MPU6050Driver::handleInput, this));
  
        });
    }


void MPU6050Driver::handleInput()
{
  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = this->get_clock()->now();
  message.header.frame_id = "base_footprint";
  message.linear_acceleration_covariance = {0};
  message.linear_acceleration.x = mpu6050_->getAccelerationX();
  message.linear_acceleration.y = mpu6050_->getAccelerationY();
  message.linear_acceleration.z = mpu6050_->getAccelerationZ();
  message.angular_velocity_covariance[0] = {0};
  message.angular_velocity.x = mpu6050_->getAngularVelocityX();
  message.angular_velocity.y = mpu6050_->getAngularVelocityY();
  message.angular_velocity.z = mpu6050_->getAngularVelocityZ();
  
  geometry_msgs::msg::TransformStamped transformStamped;
  // Calculate Euler angles
  auto accel_x = mpu6050_->getAccelerationX();
  auto accel_y = mpu6050_->getAccelerationY();
  auto accel_z = mpu6050_->getAccelerationZ();

  auto gyro_x = mpu6050_->getAngularVelocityX();
  auto gyro_y = mpu6050_->getAngularVelocityY();
  auto gyro_z = mpu6050_->getAngularVelocityZ();

  auto roll = atan2(accel_y, accel_z);
  auto pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));

  auto dt = 0.8 / this->get_parameter("frequency").as_int();
  auto gyro_roll = gyro_x * dt;
  auto gyro_pitch = gyro_y * dt;
  auto gyro_yaw = gyro_z * dt;

  // Apply complementary filter
  auto alpha = 0.98; // Adjust this value based on your system characteristics
  roll = alpha * (roll + gyro_roll) + (1.0 - alpha) * roll;
  pitch = alpha * (pitch + gyro_pitch) + (1.0 - alpha) * pitch;
  auto yaw = gyro_yaw; // You may need to integrate gyro_yaw over time for continuous yaw

  // Convert Euler angles to quaternion
  auto cy = cos(yaw * 0.5);
  auto sy = sin(yaw * 0.5);
  auto cp = cos(pitch * 0.5);
  auto sp = sin(pitch * 0.5);
  auto cr = cos(roll * 0.5);
  auto sr = sin(roll * 0.5);

  auto qw = cy * cp * cr + sy * sp * sr;
  auto qx = cy * cp * sr - sy * sp * cr;
  auto qy = sy * cp * sr + cy * sp * cr;
  auto qz = sy * cp * cr - cy * sp * sr;

  // Update the orientation in the IMU message
  message.orientation_covariance[0] = 0; // Set covariance to valid value
  message.orientation.x = qx;
  message.orientation.y = qy;
  message.orientation.z = qz;
  message.orientation.w = qw;

  publisher_->publish(message);
  
  //This part Broadcasts the transform from "base_footprint" to "imu_link"
  //geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped.header.stamp = this->now();
  transformStamped.header.frame_id = "base_footprint";
  transformStamped.child_frame_id = "imu_link";
  transformStamped.transform.translation.x = 0.0;
  transformStamped.transform.translation.y = 0.0;
  transformStamped.transform.translation.z = 0.0;
  transformStamped.transform.rotation.x = qx;
  transformStamped.transform.rotation.y = qy;
  transformStamped.transform.rotation.z = qz;
  transformStamped.transform.rotation.w = qw;
  //"Publish" or Broadcast the transform
  tf_broadcaster_->sendTransform(transformStamped);
}

void MPU6050Driver::declareParameters()
{
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<int>("gyro_range", MPU6050Sensor::GyroRange::GYR_250_DEG_S);
  this->declare_parameter<int>("accel_range", MPU6050Sensor::AccelRange::ACC_2_G);
  this->declare_parameter<int>("dlpf_bandwidth", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);
  this->declare_parameter<double>("gyro_x_offset", 0.0);
  this->declare_parameter<double>("gyro_y_offset", 0.0);
  this->declare_parameter<double>("gyro_z_offset", 0.0);
  this->declare_parameter<double>("accel_x_offset", 0.0);
  this->declare_parameter<double>("accel_y_offset", 0.0);
  this->declare_parameter<double>("accel_z_offset", 0.0);
  this->declare_parameter<int>("frequency", 0.0);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU6050Driver>());
  rclcpp::shutdown();
  return 0;
}
