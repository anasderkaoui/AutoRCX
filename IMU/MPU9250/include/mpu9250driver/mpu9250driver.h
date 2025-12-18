#ifndef MPU9250DRIVER_H
#define MPU9250DRIVER_H

#include "mpu9250sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


class MPU9250Driver : public rclcpp::Node {
 public:
  MPU9250Driver();

 private:
  std::unique_ptr<MPU9250Sensor> mpu9250_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  void handleInput();
  void declareParameters();
};

#endif  // MPU9250DRIVER_H
