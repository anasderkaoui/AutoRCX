# Important info at the end

## ROBOt's URDF description:

```xml
<?xml version="1.0"?>
<robot name="rc_car">

<!-- This is the URDF file for the rc car, it works very well, only thing is to take care of lidar-->

  <!-- Car base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.40 0.19 0.15"/>
      </geometry>
      <material name="grey">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.40 0.19 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Front left wheel -->
  <link name="front_left_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>

  <!-- Front right wheel -->
  <link name="front_right_wheel">
    <visual>
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>

  <!-- Rear left wheel -->
  <link name="rear_left_wheel">
    <visual>
<!-- <origin ryp="1.570795 0 0" xyz="0 0 0"/> -->
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>

  <!-- Rear right wheel -->
  <link name="rear_right_wheel">
    <visual>
<!-- <origin ryp="1.570795 0 0" xyz="0 0 0"/> -->
      <geometry>
        <cylinder length="0.05" radius="0.03"/>
      </geometry>
      <material name="black">
        <color rgba="0.1 0.1 0.1 1"/>
      </material>
    </visual>
  </link>

  <!-- LiDAR sensor -->
  <link name="lidar">
    <visual>
      <geometry>
        <cylinder length="0.02" radius="0.035"/>
      </geometry>
      <material name="grey">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
  </link>

  <!-- Intermediate steering links -->
  <link name="front_left_wheel_steering"/>
  <link name="front_right_wheel_steering"/>

  <!-- Joints -->

  <!-- Front left wheel steering joint (yaw) -->
  <joint name="front_left_steering_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_wheel_steering"/>
    <origin xyz="0.15 0.09 -0.075" rpy="1.570795 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
  </joint>

  <!-- Front right wheel steering joint (yaw) -->
  <joint name="front_right_steering_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_right_wheel_steering"/>
    <origin xyz="0.15 -0.09 -0.075" rpy="1.570795 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1.0"/>
    <mimic joint="front_left_steering_joint" multiplier="1.0" offset="0"/> <!-- wheel mimics the other wheel as if they were connected -->
  </joint>

  <!-- Front left wheel rotation joint (rolling) -->
  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="front_left_wheel_steering"/>
    <child link="front_left_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Front right wheel rotation joint (rolling) -->
  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="front_right_wheel_steering"/>
    <child link="front_right_wheel"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <mimic joint="front_left_wheel_joint" multiplier="1.0" offset="0"/> <!-- wheel mimics the other wheel as if they were connected -->
  </joint>

  <!-- Rear left wheel joint (rolls == continuous, fixed == fixed) -->
  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.15 0.09 -0.075" rpy="1.570795 0 0"/>
    <axis xyz="0 0 1" />
    <mimic joint="front_left_wheel_joint" multiplier="1.0" offset="0"/> <!-- wheel mimics the other wheel as if they were connected -->
  </joint>

  <!-- Rear right wheel joint (rolls) -->
  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.15 -0.09 -0.075" rpy="1.570795 0 0"/>
    <axis xyz="0 0 1" />
    <mimic joint="rear_left_wheel_joint" multiplier="1.0" offset="0"/> <!-- wheel mimics the other wheel as if they were connected -->
  </joint>

  <!-- LiDAR joint (fixed) -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar"/>
    <origin xyz="0 0 0.085" rpy="0 0 0"/>
  </joint>

</robot>

```


Node (to be tested) to synchronize real robot motion to joint states:

```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

// Robot dimensions (adjust to match your robot)
const double WHEELBASE = 0.254; // Distance between front and rear axles (meters)
const double TRACK_WIDTH = 0.167; // Distance between left and right wheels (meters)
const double WHEEL_RADIUS = 0.05; // Radius of the wheels (meters)

class AckermanSimulationNode {
public:
    AckermanSimulationNode() {
        // Initialize subscribers and publishers
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 10, &AckermanSimulationNode::cmdVelCallback, this);
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states", 10);

        // Initialize joint state message
        joint_state_msg_.name = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint", "rear_right_wheel_joint", "front_left_steering_joint", "front_right_steering_joint"};
        joint_state_msg_.position.resize(6, 0.0);
        joint_state_msg_.velocity.resize(6, 0.0);
    }

    void spin() {
        ros::Rate rate(50); // 50 Hz
        while (ros::ok()) {
            ros::spinOnce();
            updateJointStates();
            joint_state_pub_.publish(joint_state_msg_);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    void updateJointStates() {
        // Compute steering angles for Ackerman steering
        double delta_left = 0.0;
        double delta_right = 0.0;

        if (angular_velocity_ != 0.0) {
            double turning_radius = linear_velocity_ / angular_velocity_;
            delta_left = std::atan(WHEELBASE / (turning_radius - (TRACK_WIDTH / 2.0)));
            delta_right = std::atan(WHEELBASE / (turning_radius + (TRACK_WIDTH / 2.0)));
        }

        // Update wheel joint positions (steering)
        joint_state_msg_.position[4] = delta_left;  // front_left_steering_joint
        joint_state_msg_.position[5] = delta_right; // front_right_steering_joint

        // Compute wheel velocities
        double rear_wheel_velocity = linear_velocity_ / WHEEL_RADIUS;

        joint_state_msg_.velocity[0] = rear_wheel_velocity; // front_left_wheel_joint
        joint_state_msg_.velocity[1] = rear_wheel_velocity; // front_right_wheel_joint
        joint_state_msg_.velocity[2] = rear_wheel_velocity; // rear_left_wheel_joint
        joint_state_msg_.velocity[3] = rear_wheel_velocity; // rear_right_wheel_joint
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackerman_simulation_node");
    AckermanSimulationNode node;
    node.spin();
    return 0;
}
```

Full info about **car dimensions**: 25,4cm wheel separation from front to back wheels 16,7cm two-wheel separation (all taking from center of wheel/tire) !! CAR 47,5cm total length and 20cm total boarder-to-boarder large!

## Autonmous Navigation Fixes/Requirements

> [!NOTE]
> The **major fixes** to the autnomous navigation of the AutoRCX are:<br>
> **1-** The inflation layer activation should be noted in either the "local_costmap" or "common_costmap" so that the robot can detect dynamic obstacles in real time and update its trajectory. See the corrected version HERE*.<br>
> **2-** LiDAR scans were moving along with the car because there was **no Odometry topic/data published!!** Odometry should be present or calculated in order for AMCL to calculate and the update the translation motion. However this is not enough!<br>
> **3-** On top of the odometry, there has to be an IMU in order to calculate the rotation and orientation of the robot. Best approach is to fuse odometry and IMU (Extended Kalman filter) to give AMCL a better approximation. The IMU especially helps correct robot position while rotating.<br>
> **4-** There are some preferred choises for Ackerman (car-like) robots in navigation. Carrot planner as global planner and TEB planner as local planner. Both present under the "move_base" package!<br>

Lastly, I have noticed a strange behaviour lately, especially while adding the kalman filter node. The EKF publishes a "tf", at the reception of the "IMU" and "odom" nodes, and it needs the URFD of the robot in order to publish exactly the position of each component of the robot.<br>
I think the issue that I had was related to the fact that I was not publishing the URDF and this explains the weird behavior that I had gotten when I launched the navigation stack. The strange behavior observed: the transforms would appear everywhere and jump all over the map and even outside it, in RVIZ! So since there was no description of the car components, the filter kept giving random places and positions to the transforms. So, the **fix that needs to be made, is to add/uncomment the URDF in the launch file! Assuming it works well and has all the necessary tf**
