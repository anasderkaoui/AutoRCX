#include "mpu9250driver/mpu9250driver.h"
#include <chrono>
#include <memory>
#include "LinuxI2cCommunicator.h"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

#include <cmath>

// Normalize yaw to 0-360°
double normalize_angle_360(double angle_deg) {
    while (angle_deg < 0) angle_deg += 360.0;
    while (angle_deg >= 360.0) angle_deg -= 360.0;
    return angle_deg;
}

MPU9250Driver::MPU9250Driver() : Node("mpu9250publisher")
{
    // Create concrete I2C communicator and pass to sensor
    std::unique_ptr<I2cCommunicator> i2cBus = std::make_unique<LinuxI2cCommunicator>();
    mpu9250_ = std::make_unique<MPU9250Sensor>(std::move(i2cBus));
    // Declare parameters
    declareParameters();

    // Set parameters
    mpu9250_->setGyroscopeRange(
        static_cast<MPU9250Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
    mpu9250_->setAccelerometerRange(
        static_cast<MPU9250Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
    mpu9250_->setDlpfBandwidth(
        static_cast<MPU9250Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
    mpu9250_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                                 this->get_parameter("gyro_y_offset").as_double(),
                                 this->get_parameter("gyro_z_offset").as_double());
    mpu9250_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                     this->get_parameter("accel_y_offset").as_double(),
                                     this->get_parameter("accel_z_offset").as_double());

    // Check if we want to calibrate the sensor
    if (this->get_parameter("calibrate").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "Calibrating...");
      mpu9250_->calibrate();
    }
    mpu9250_->printConfig();
    mpu9250_->printOffsets();

    // Publishers for raw IMU and magnetometer
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", 10);
    mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Set timer frequency from parameter (Hz)
    int frequency = this->get_parameter("frequency").as_int();
    if (frequency <= 0) frequency = 100; // Default to 100 Hz
    auto timer_period = std::chrono::milliseconds(1000 / frequency);
    timer_ = this->create_wall_timer(timer_period, std::bind(&MPU9250Driver::handleInput, this));
}

void MPU9250Driver::declareParameters()
{
    // No need to calibrate accel, already done in mpu9250sensor.cpp
    this->declare_parameter<bool>("calibrate", true);
    this->declare_parameter<int>("gyro_range", MPU9250Sensor::GyroRange::GYR_250_DEG_S);
    this->declare_parameter<int>("accel_range", MPU9250Sensor::AccelRange::ACC_2_G);
    this->declare_parameter<int>("dlpf_bandwidth", MPU9250Sensor::DlpfBandwidth::DLPF_260_HZ);
    this->declare_parameter<double>("gyro_x_offset", 0.0);
    this->declare_parameter<double>("gyro_y_offset", 0.0);
    this->declare_parameter<double>("gyro_z_offset", 0.0);
    this->declare_parameter<double>("accel_x_offset", 0.0);
    this->declare_parameter<double>("accel_y_offset", 0.0);
    this->declare_parameter<double>("accel_z_offset", 0.0);
    this->declare_parameter<int>("frequency", 100); // default 100 Hz
}

void MPU9250Driver::handleInput()
{
    // ==== IMU MESSAGE ====
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";

    // Read sensor data
    double ax = mpu9250_->getAccelerationX();
    double ay = mpu9250_->getAccelerationY();
    double az = mpu9250_->getAccelerationZ();

    double gx = mpu9250_->getAngularVelocityX();
    double gy = mpu9250_->getAngularVelocityY();
    double gz = mpu9250_->getAngularVelocityZ();

    mpu9250_->readMagnetometer(); // Make sure data is updated!
    double mx = mpu9250_->getMagneticFluxDensityX();
    double my = mpu9250_->getMagneticFluxDensityY();
    double mz = mpu9250_->getMagneticFluxDensityZ();

    // Set raw IMU data
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.linear_acceleration_covariance[0] = 0;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;
    imu_msg.angular_velocity_covariance[0] = 0;

    // // ----------- TILT COMPENSATED YAW -----------
    // // Calculate roll and pitch from accel (in radians)
    // double roll  = atan2(ay, az);
    // double pitch = atan(-ax / (ay*sin(roll) + az*cos(roll) + 1e-8)); // +1e-8 to avoid div by zero

    // // Tilt compensation
    // double mag_x_comp = mx * cos(pitch) + mz * sin(pitch);
    // double mag_y_comp = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);

    // // Calculate yaw (in radians)
    // double yaw = atan2(-mag_y_comp, mag_x_comp);
    // // ----------- TILT COMPENSATED YAW END-----------

    // ----------- YAW WITHOUT TILT COMPENSATION -----------
    // Only use magnetometer X and Y (no accel!)
    double yaw = atan2(-my, mx); // or atan2(my, mx), depending on your axis
    // ----------- YAW WITHOUT TILT COMPENSATION END-----------

    // Convert to quaternion (use roll=0, pitch=0 if you only want yaw, or use computed roll/pitch for better orientation)
    // Here: set roll and pitch = 0 (useful for mobile robots on a flat plane)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    // double cp = cos(pitch / 2.0); // Uncomment if using yaw tilt compensation
    // double sp = sin(pitch / 2.0);
    // double cr = cos(roll / 2.0);
    // double sr = sin(roll / 2.0);

    // imu_msg.orientation.w = cr * cp * cy + sr * sp * sy; // Uncomment if using yaw tilt compensation
    // imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
    // imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
    // imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
    // imu_msg.orientation_covariance[0] = 0; // now orientation is valid

    imu_msg.orientation.w = cy; // Comment if using yaw tilt compensation
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = sy;

    // Covariances
    // MAG
    imu_msg.orientation_covariance[0] = -1.0;  // roll  // Lower values = higher trust (EKF believes more your measurement).
    imu_msg.orientation_covariance[4] = -1.0;  // pitch   // Higher values = lower trust (EKF relies more on other data or prediction).
    imu_msg.orientation_covariance[8] = 0.05;  // yaw
    // GYRO
    imu_msg.angular_velocity_covariance[0] = -1.0;  //roll rate // -1 means don't take into account !
    imu_msg.angular_velocity_covariance[4] = -1.0;  //pitch rate
    imu_msg.angular_velocity_covariance[8] = 0.05;  //yaw rate
    // ACCEL
    imu_msg.linear_acceleration_covariance[0] = 0.05;  // X
    imu_msg.linear_acceleration_covariance[4] = 0.05;  // Y
    imu_msg.linear_acceleration_covariance[8] = -1.0;  // Z

    imu_publisher_->publish(imu_msg);

    // ==== MAGNETOMETER MESSAGE ====
    auto mag_msg = sensor_msgs::msg::MagneticField();
    mag_msg.header = imu_msg.header;
    mag_msg.magnetic_field.x = mx;
    mag_msg.magnetic_field.y = my;
    mag_msg.magnetic_field.z = mz;

    mag_publisher_->publish(mag_msg);

    // Prepare TransformStamped
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "base_link";
    t.child_frame_id = "imu_link";
    t.transform.translation.x = 0.0;      // Change if IMU is offset!
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = imu_msg.orientation.x;
    t.transform.rotation.y = imu_msg.orientation.y;
    t.transform.rotation.z = imu_msg.orientation.z;
    t.transform.rotation.w = imu_msg.orientation.w;
    // Send the transform
    tf_broadcaster_->sendTransform(t);

    // --- Optional: Print yaw for debugging ---
    // double yaw_deg = yaw * 180.0 / M_PI;
    // yaw_deg = normalize_angle_360(yaw_deg);
    // std::cout << "[YAW raw]: " << yaw_deg << "° ";

    // double yaw_non_tilt = atan2(-my, mx);
    // double yaw_non_tilt_deg = yaw_non_tilt * 180.0 / M_PI;
    // if (yaw_non_tilt_deg < 0) yaw_non_tilt_deg += 360.0;
    // std::cout << "[YAW non-tilt]: " << yaw_non_tilt_deg << "°  (raw mag, no tilt compensation)" << std::endl;

    // // No orientation computed here! Waiting for a filter like Madgwick or Mahony to calculate it !
    // imu_msg.orientation.x = 0.0;
    // imu_msg.orientation.y = 0.0;
    // imu_msg.orientation.z = 0.0;
    // imu_msg.orientation.w = 1.0;
    // imu_msg.orientation_covariance[0] = -1; // -1 = unknown/orientation not provided

    // std::cout << "Mag raw: " 
    // << mpu9250_->getMagneticFluxDensityX() << ", "
    // << mpu9250_->getMagneticFluxDensityY() << ", "
    // << mpu9250_->getMagneticFluxDensityZ() << std::endl;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPU9250Driver>());
  rclcpp::shutdown();
  return 0;
}
