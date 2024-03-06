This is a complete tutorial to do Simultanuous Localization and Mapping using:
- YDLIDAR G4
- MPU6050 IMU
- DYNAMIXEL MX-106 Servomotor (This can also work for other* servomotors from DYNAMIXEL)

The SLAM algorithm (**slam_toolbox**) is running on a Raspberry Pi4, Ubuntu 22.04, ROS 2 Humble.

## First:
Download the corresponding ROS 2 drivers for each one of the pieces of hardware above. For the IMU you can simply clone [this folder available within this repo](https://github.com/anasderkaoui/AutoRCX/tree/main/IMU/MPU6050/ros2_mpu6050).
ROS 2 Drivers for the:
- [YDLIDAR G4](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
- [DYNAMIXEL MX-106](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel)


### Keywords: SLAM, Odometry, Lidar, TF, map, base_footprint, IMU.
*: Different models were tested but not all of them.
