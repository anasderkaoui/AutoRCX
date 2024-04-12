This is a complete tutorial to do Simultanuous Localization and Mapping using:
- YDLIDAR G4
- MPU6050 IMU
- DYNAMIXEL MX-106 Servomotor (This can also work for other* servomotors from DYNAMIXEL)

The SLAM algorithms (**slam_toolbox and slam_gmapping**) have been tested on a **Raspberry Pi4, Ubuntu 22.04, ROS 2 Humble** and also on **Ubuntu 20.04, ROS 2 Foxy**.

## First:
Download the corresponding ROS 2 drivers for each one of the pieces of hardware above. For the IMU you can simply clone [this folder available within this repo](https://github.com/anasderkaoui/AutoRCX/tree/main/IMU/MPU6050/ros2_mpu6050).
ROS 2 Drivers for the:
- [YDLIDAR G4](https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble)
- [DYNAMIXEL MX-106](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/humble-devel)



# SLAM configuration
This package is used to perform Simultanuous Localization and Mapping (SLAM).<br>
Each sub-package contains a more detailed documentation for dependency installation.<br>
Here, you will find:

**IMU folder**: "imu_ws"<br>
-> IMU Model 1: **MPU6050**
- A package for angle extraction from an IMU and for visualization on RVIZ. The angles that will be displayed for now are "Roll" and "Pitch". There is a complimentary filter that will help to do "Yaw" calculation, but the output will never be the absolute angle and will have a lot of noise.
- SETUP:<br>

<p align="center">
    <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/2e9da7e4-1e17-41ea-974b-14a80d6caa84">

- Launch it separately: `sudo chmod +777 /dev/i2c-* && ros2 run mpu6050_driver mpu6050_driver_node`

-> IMU Model 2: **BNO055**
- This IMU, unlike the previous one, has integrated calculations. This means that Euler Angles and Quaternions are already calculated by the IMU and we only need the driver that will display the ouput.
- In order to use this IMU, make sure to connect it to the raspberry Pi4 just like the previous IMU. This type of connection is for **I2C communication**.
- To use this IMU with a computer via **USB communication** (using the I2C to USB (TTL USB) converter), make sure to connect RX➜SDA, TX➜SCL, Vin➜Vcc, Gnd➜Gnd and **connect the 3V pin to the PS1 pin**:
  <p align="center">
      <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/fbe41cb9-2cef-43d6-9675-0626d6e7d087">
- [This is the ros2 driver](https://github.com/flynneva/bno055.git) for it.
- Make sure you modify [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/launch/bno055.launch.py#L38) in the launch file to "bno055_params_i2c.yaml".
- Also change the "i2c_bus" in [this line](https://github.com/flynneva/bno055/blob/45e1ff16936101711260c9fda63fbad99376ce3b/bno055/params/bno055_params_i2c.yaml#L34) of the param file to the corresponding one (usually 0 or 1).
- Install required dependencies: `rosdep install --from-paths src -i`
- Finally run: `ros2 launch bno055 bno055.launch.py`

**LiDAR folder**: "lidar_ws"<br>
-> LiDAR Model: YDLIDAR G4
- This ROS 2 package is responsible for laser scan. It starts the LiDAR and can also visualize the scanned data.
- SETUP: Connect the LiDAR to the board. It can also work on the "Data" cable alone.
- Make sure that the [YDLIDAR SDK](https://github.com/YDLIDAR/YDLidar-SDK?tab=readme-ov-file#installation) is installed before continuing, otherwise the build will not succeed. Check the "lidar_ws" package for more information.
- **WARNING: The LiDAR should be mounted on a steady surface in order for the SLAM to work properly. Small vibrations can impact significantly the generated map!**
- Launch it separately: `sudo chmod +777 /dev/ttyUSB* && ros2 launch ydlidar_driver ydlidar_launch_view.py`

> [!NOTE]
>**Make sure, after launching these packages and before launching the SLAM, that you have this Transform (tf) tree: `odom ➜ base_footprint ➜ (lidar_frame and imu_link)`**. A transform tree represents the tree of the frames of each electronic component of the robot. The frame of the wheels is called "odom", the frame of the robot (reference frame) is called either "base_footprint" or "base_link". The only difference between these two nominations is just that a part of the ROS community chooses one name and the other part chooses the other name. The frame of the IMU is called "imu_link" and finally the frame of the LiDAR is called "lidar_frame" and can also be "laser_frame". You can check the active tree by running: `ros2 run tf2_tools view_frames.py`
<p align="center">
      <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/08b7b794-b21f-4cc5-bfd9-b6e23134bc5b">

**SLAM folder**: "slam_gmapping", clone it directly from [this link](https://github.com/Project-MANAS/slam_gmapping)<br>
- This is the package that allows to perform SLAM. In order for it to work, you have to have the following nodes already running: 'odom' from the robot and 'scan' from the lidar.
- Before building it, make sure to change ["base_link"](https://github.com/Project-MANAS/slam_gmapping/blob/3c3de50c071d2c64ffe516e1ed84a574fc447b97/slam_gmapping/src/slam_gmapping.cpp#L62) to "base_footprint" if you will use the packages described above.
- Launch it separatly: `ros2 launch slam_gmapping slam_gmapping.launch.py`

There is another SLAM algorithm that is similar to 'slam_gmapping' which is **"slam_toolbox"**. It has more features than the other one, but "slam_toolbox" updates the map **only when odometry data has changed (robot has moved), unlike "slam_gmapping" which updates the map continuously**.
- To install "slam_toolbox": `sudo apt update && sudo apt install ros-$ROS_DISTRO-slam-toolbox`
- Launch it separately: `ros2 launch slam_toolbox online_sync_launch.py` ➜ There are other options, "ros2 launch slam_toolbox" and double-tap on the TAB key to see them.

**RVIZ output**: Now, in order to visualize the map generated, open RVIZ by running `rviz2`, in the "Displays" pannel ➜ "Add" ➜ "by topic" and then select "map".
<p align="center">
      <img src="https://github.com/MecaBotiX/m3cooper_ros_2/assets/115218309/d626502a-a16f-4877-a7f8-5cfbff3ae2dc">
    
# FAQ
- Error of type: "Cannot retrieve YDLIDAR health" while running the LiDAR or any related error ➜ **Make sure to specify the right port of the LiDAR in the ["yaml" file](https://github.com/MecaBotiX/m3cooper_ros_2/blob/f3ffba6bb7bf43f59c6fc5fd2a0007cef9da1ffb/SLAM/lidar_ws/src/ydlidar_ros2_driver/params/ydlidar.yaml#L3)**.
- Error of type: "Discarding message because queue is full" on terminal and the output on RVIZ is stuck ➜ **This indicates that the queue holding the received messages is full. Go to the corresponding package and increase the size of the queue.**
- Map overlapping problem ➜ Make sure the LiDAR is placed on a stable support and is not vibrating or shaking or making sudden moves and rotations. The LiAR should be moving very smoothly and slowly in order for the SLAM algorithm to generate a smooth single map.

### Keywords: SLAM, Odometry, Lidar, TF, map, base_footprint, IMU.
*Different models were tested but not all of them.
