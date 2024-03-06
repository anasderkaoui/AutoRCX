#ifndef MPU6050DRIVER_H
#define MPU6050DRIVER_H

#include "mpu6050driver/mpu6050sensor.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "Eigen/Geometry"
#include <tf2/convert.h>

class MPU6050Driver : public rclcpp::Node {
 public:
  MPU6050Driver();

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::unique_ptr<MPU6050Sensor> mpu6050_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  size_t count_;
  rclcpp::TimerBase::SharedPtr timer_;
  void handleInput();
  void declareParameters();
};

#endif  // MPU6050DRIVER_H
