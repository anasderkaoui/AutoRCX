This is a ROS 2 package to communicate and show the MPU6050 IMU output on RVIZ2 using I2C interface. The sensor is calibrated on node startup (sensor needs to be on a plane with z-axis up and should not be moved during calibration). Calibration can be turned off in the parameters (.yaml) file. The output is an IMU ROS message and the quaternion part is being calculated in order to show Roll and Pitch angles, Yaw is an approximate value and not the absolute value.

### Dependencies (should be installed to build and run the package)
-  libi2c-dev

   `sudo apt install libi2c-dev`

Create a workspace and go there:

    mkdir -p ros2_imu_ws/src && cd ros2_imu_ws/src

Copy this folder to your workspace "src"

Build the package in your workspace:

    cd ros2_imu_ws/ && colcon build --packages-select mpu6050driver

Source your workspace:

    . install/setup.bash

Give permission to the IMU for data transfer:

    sudo chmod +777 /dev/i2c-*
    
Launch the code:

    ros2 launch mpu6050driver mpu6050driver_launch.py
