Create a workspace and go there:

    mkdir -p ros2_imu_ws/src && cd ros2_imu_ws/src

Clone this folder:

    git clone 

Build the package in your workspace:

    colcon build --packages-select mpu6050driver

Source setup.bash in your workspace:

    . install/setup.bash
    
Launch it:

    ros2 launch mpu6050driver mpu6050driver_launch.py
