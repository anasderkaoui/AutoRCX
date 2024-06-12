In this report, we will see how can we use the lidar in order to do some SLAM.

The first step is to setup our ROS environment. In oder to do that follow these simple steps:

1- 





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

  # Trajectory scoring parameters
  sim_time: 1.0
  sim_granularity: 0.025
  vx_samples: 6
  vtheta_samples: 20
  path_distance_bias: 32.0
  goal_distance_bias: 24.0
  occdist_scale: 0.01
  forward_point_distance: 0.325

  # Oscillation prevention
  oscillation_reset_dist: 0.05
  escape_reset_dist: 0.10
  escape_reset_theta: 0.10
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

Launch file:
```
<launch>
  <!-- Map Server -->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find your_package)/maps/my_map.yaml" />

  <!-- AMCL -->
  <node name="amcl" pkg="amcl" type="amcl" output="screen">
    <param name="odom_frame_id" value="odom"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
    <param name="odom_model_type" value="diff"/>
    <rosparam file="$(find your_package)/config/amcl_params.yaml" command="load"/>
  </node>

  <!-- Move Base -->
  <node name="move_base" pkg="move_base" type="move_base" output="screen">
    <rosparam file="$(find your_package)/config/global_costmap_params.yaml" command="load" ns="global_costmap"/>
    <rosparam file="$(find your_package)/config/local_costmap_params.yaml" command="load" ns="local_costmap"/>
    <rosparam file="$(find your_package)/config/base_local_planner_params.yaml" command="load"/>
  </node>
</launch>
```
CMkaeFile:
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
