In this report, we will see how can we use the lidar in order to do some SLAM.

The first step is to setup our ROS environment. In oder to do that follow these simple steps:

1- `source /opt/ros/$ROS_DISTRO/setup.bash`
2- Packages setup:

The mistake I was doing is running SLAM along navigation which is not right because two subscribers to map are present and the navigation stack doesn't like it!!!!
The right thing to do is to publish a static transform publisher from base_link to laser. And the "map" to "base_link" frame will be published automatically by the map server from the previously saved map !! Now when solicited, the warning "could not get robot pose" seems to be gone!

Interest in SLAM Filter Sampling to fix map overlapping.

- LiDAR package
- Hector mapping (SLAM) package
- Navigation package
- In my case I was using an Arduino UNO that controls the servo motor and the DC motor of my autonomous RC car. I had to configure other packages:
  - rosserial. Command to initialize communication: `rosserial `
  - Arduino code and test code.

global_costmap_params.yaml:
```
global_costmap:
  global_frame: "map"
  robot_base_frame: "base_link"
  update_frequency: 1.0
  publish_frequency: 0.5
  transform_tolerance: 0.5
  static_map: true

  width: 10.0
  height: 10.0
  resolution: 0.05
  origin_x: 0.0
  origin_y: 0.0
  rolling_window: false

  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  footprint: [[-0.20, -0.09], [-0.20, 0.09], [0.20, 0.09], [0.20, -0.09]]  # Approx. 40cm x 18cm
  footprint_padding: 0.01  # Extra padding around the footprint for safety
```

local_costmap_params.yaml:
```
local_costmap:
  global_frame: "odom"
  robot_base_frame: "base_link"
  update_frequency: 5.0
  publish_frequency: 2.0
  transform_tolerance: 0.5
  rolling_window: true

  width: 3.0
  height: 3.0
  resolution: 0.05

  plugins:
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  footprint: [[-0.20, -0.09], [-0.20, 0.09], [0.20, 0.09], [0.20, -0.09]]  # Approx. 40cm x 18cm
  footprint_padding: 0.01  # Extra padding around the footprint for safety
```

base_local_planner_params.yaml :
```
TrajectoryPlannerROS:
  max_vel_x: 0.5
  min_vel_x: 0.1
  max_vel_theta: 1.0
  min_vel_theta: -1.0
  acc_lim_x: 1.0
  acc_lim_theta: 1.0
  xy_goal_tolerance: 0.1
  yaw_goal_tolerance: 0.1
  latch_xy_goal_tolerance: true

  sim_time: 1.0
  sim_granularity: 0.025
  vx_samples: 6
  vtheta_samples: 20
  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  occdist_scale: 0.01
  forward_point_distance: 0.325

  oscillation_reset_dist: 0.05
  escape_reset_dist: 0.10
  escape_reset_theta: 0.10

  footprint: [[-0.20, -0.09], [-0.20, 0.09], [0.20, 0.09], [0.20, -0.09]]  # Approx. 40cm x 18cm
  footprint_padding: 0.01  # Extra padding around the footprint for safety
```

amcl_params.yaml:

```
amcl:
  min_particles: 100
  max_particles: 500
  kld_err: 0.05
  update_min_d: 0.2
  update_min_a: 0.2
  resample_interval: 2
  transform_tolerance: 0.5
  recovery_alpha_slow: 0.001
  recovery_alpha_fast: 0.1
  tf_broadcast: true
  odom_frame_id: "odom"
  base_frame_id: "base_link"
  global_frame_id: "map"
  laser_lambda_short: 0.1
  laser_likelihood_max_dist: 2.0
  laser_model_type: "likelihood_field"
  odom_model_type: "diff"
```

CMakeFile:
```
cmake_minimum_required(VERSION 2.8.3)
project(your_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  move_base
  amcl
  map_server
  costmap_2d
  base_local_planner
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs tf move_base amcl map_server costmap_2d base_local_planner
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

xml file:
```
<?xml version="1.0"?>
<package format="2">
  <name>your_package</name>
  <version>0.0.0</version>
  <description>The your_package package</description>

  <maintainer email="your_email@example.com">your_name</maintainer>

  <license>BSD</license>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>geometry_msgs</build_depend>
  <build_depend>sensor_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf</build_depend>
  <build_depend>move_base</build_depend>
  <build_depend>amcl</build_depend>
  <build_depend>map_server</build_depend>
  <build_depend>costmap_2d</build_depend>
  <build_depend>base_local_planner</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf</exec_depend>
  <exec_depend>move_base</exec_depend>
  <exec_depend>amcl</exec_depend>
  <exec_depend>map_server</exec_depend>
  <exec_depend>costmap_2d</exec_depend>
  <exec_depend>base_local_planner</exec_depend>

  <export>
    <build_type>catkin</build_type>
  </export>
</package>

```

Car's URDF or XACRO:
```
<?xml version="1.0"?>
<robot name="rc_car">

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
        <cylinder length="0.04" radius="0.02"/>
      </geometry>
      <material name="grey">
        <color rgba="0.6 0.6 0.6 1"/>
      </material>
    </visual>
  </link>

  <!-- Joints -->
  <!-- Front left wheel joint -->
  <joint name="front_left_wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.15 0.09 -0.075" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="10"/>
  </joint>

  <!-- Front right wheel joint -->
  <joint name="front_right_wheel_joint" type="revolute">
    <parent link="base_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0.15 -0.09 -0.075" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="10"/>
  </joint>

  <!-- Rear left wheel joint -->
  <joint name="rear_left_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.15 0.09 -0.075" rpy="0 0 0"/>
  </joint>

  <!-- Rear right wheel joint -->
  <joint name="rear_right_wheel_joint" type="fixed">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.15 -0.09 -0.075" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR joint -->
  <joint name="lidar_joint" type="fixed">
    <parent link="base_link"/>
    <child link="lidar"/>
    <origin xyz="0 0 0.20" rpy="0 0 0"/>
  </joint>

</robot>
```

RVIZ config file, can also be generated automatically just by saving the config before exiting RVIZ:
```
Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views
Visualization Manager:
  Class: ""rviz::VisualizationManager""
  Displays:
    - Class: rviz/Grid
      Name: Grid
    - Class: rviz/RobotModel
      Name: RobotModel
      Robot Description: robot_description
    - Class: rviz/Map
      Name: Global Costmap
      Topic: /move_base/global_costmap/costmap
      Alpha: 0.7
    - Class: rviz/Map
      Name: Local Costmap
      Topic: /move_base/local_costmap/costmap
      Alpha: 0.7
    - Class: rviz/LaserScan
      Name: LaserScan
      Topic: /scan
    - Class: rviz/TF
      Name: TF
    - Class: rviz/PoseArray
      Name: ParticleCloud
      Topic: /particlecloud
    - Class: rviz/Path
      Name: GlobalPlan
      Topic: /move_base/GlobalPlanner/plan
  Fixed Frame: map
  Value: "RViz Config"

Views:
  - Class: rviz/Orbit
    Name: Orbit
    Value: "Orbit View"
```

Launch file:
```
<launch>
  <!-- Load the URDF -->
  <param name="robot_description" command="cat $(find your_package)/urdf/rc_car.urdf"/>

  <!-- Hector Mapping -->
  <node name="hector_mapping" pkg="hector_mapping" type="hector_mapping" output="screen"/>

  <!-- Map server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find your_package)/maps/map.yaml"/>

  <!-- AMCL node -->
  <node name="amcl" pkg="amcl" type="amcl" output="screen">
    <param name="use_map_topic" value="true"/>
    <param name="odom_frame_id" value="map"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
    <param name="laser_model_type" value="likelihood_field"/>
    <param name="odom_model_type" value="diff"/>
    <param name="update_min_d" value="0.2"/>
    <param name="update_min_a" value="0.2"/>
    <param name="min_particles" value="100"/>
    <param name="max_particles" value="500"/>
    <param name="kld_err" value="0.05"/>
    <param name="resample_interval" value="2"/>
    <param name="transform_tolerance" value="0.5"/>
  </node>

  <!-- Static transform for laser -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0.1 0 0 0 0 0 base_link laser 100"/>

  <!-- Move base -->
  <include file="$(find your_navigation_package)/launch/move_base.launch"/>

  <!-- RViz -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find your_package)/rviz/your_config.rviz"/>
</launch>

```
