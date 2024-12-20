This is a ROS 2 package to communicate and show the MPU6050 IMU output on RVIZ2 using I2C interface. The sensor is calibrated on node startup (sensor needs to be on a plane with z-axis up and should not be moved during calibration). Calibration can be turned off in the parameters (.yaml) file. The output is an IMU ROS message and the quaternion part is being calculated in order to show Roll and Pitch angles, Yaw is an approximate value and not the absolute value.

> [!IMPORTANT]
> **Transform tree explanation** <br> A transform tree (or **TF**) represents the tree of the frames of each electronic component of the robot. The frame of the frame of the robot (reference frame) is called either **"base_footprint" or "base_link"**. **"base_footprint"** is the projection of the robot on a 2D plane on the ground, it might be useful when the robot is placed higher on the ground or maybe when it is suspended in the air for a certain reason. **"base_link"** is the frame attached to the robot, the frame of the robot. (~~The only difference between these two nominations is just that a part of the ROS community chooses one name and the other part chooses the other name.~~). The frame of the IMU is called **"imu_link"**. You can check the active tree by running: `ros2 run tf2_tools view_frames.py` <br>
RVIZ2 needs the transform tree in order to know which is the base of reference and which frame is to be translated or rotated with respect to that base reference.

### Dependencies (should be installed to build and run the package)
-  libi2c-dev

   `sudo apt install libi2c-dev`

Create a workspace and go there:

    mkdir -p ros2_imu_ws/src && cd ros2_imu_ws/src

Copy this folder to your workspace "src"

Build the package in your workspace:

    cd ros2_imu_ws/ && colcon build --packages-select mpu6050driver

You might run into this error (for FOXY users): **tf2_geometry_msgs/tf2_geometry_msgs.hpp not found** ! <br>FIX -> `cd src/ros2_mpu6050_driver/include/mpu6050driver/ && gedit mpu6050driver.h` -> Modify the extension of [the include line](https://github.com/anasderkaoui/AutoRCX/blob/a9357d526fd1dec3f59eb3526fe910289f3911a2/IMU/MPU6050/ros2_mpu6050/include/mpu6050driver.h#L11) of tf2_geometry from ".hpp" to ".h"

Source your workspace:

    . install/setup.bash

Give permission to the IMU for data transfer:

    sudo chmod +777 /dev/i2c-*
    
Launch the code:

    ros2 launch mpu6050driver mpu6050driver_launch.py

Visualize the IMU on RVIZ2:

    rviz2
- After openning RVIZ2, at the left pannel, click on "*Add*". Under the "*By display type*" option, choose "*rviz_imu_plugin/Imu*" and click on "*Ok*".<br>

  ![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/f3e9c253-009c-4423-beef-10ecd6c586b9)

- Now, always on the left pannel select "*base_link*" in the "*Fixed Frame*" tab.<br>
- Next, click on the "*Imu*" tab and in the tab in front of "*Topic*", double click and choose "*imu_link*".<br>
![image](https://github.com/anasderkaoui/AutoRCX/assets/115218309/f2b62bca-8e71-43b4-ae0e-7a8fe7f36b7c)<br>
- Now you should be able to visualize the IMU without a problem.
