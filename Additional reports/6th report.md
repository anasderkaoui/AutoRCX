In this report, we will see how can we use the lidar in order to do some SLAM.

The first step is to setup our ROS environment. In oder to do that follow these simple steps:

1- `source /opt/ros/$ROS_DISTRO/setup.bash`
2- Packages setup:

The mistake I was doing is running SLAM along navigation which is not right because two subscribers to map are present and the navigation stack doesn't like it!!!!
The right thing to do is to publish a static transform publisher from base_link to laser. And the "map" to "base_link" frame will be published automatically by the map server from the previously saved map !! Now when solicited, the warning "could not get robot pose" seems to be gone!

To fix map overlapping, stabilize LiDAR and avoid sharp turns.

- LiDAR package
- Hector mapping (SLAM) package
- Navigation package
- In my case I was using an Arduino UNO that controls the servo motor and the DC motor of my autonomous RC car. I had to configure other packages:
  - rosserial. Command to initialize communication: `rosserial`
  - Arduino code and test code.

In global costmap, no need to define origins, the plugin will define the map uploaded as global one!
global_costmap_params.yaml:
```yaml
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
  rolling_window: false

  plugins:
    - {name: static_layer, type: "costmap_2d::StaticLayer"}
    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}

  footprint: [[-0.20, -0.09], [-0.20, 0.09], [0.20, 0.09], [0.20, -0.09]]  # Approx. 40cm x 18cm
  footprint_padding: 0.01  # Extra padding around the footprint for safety
```

new global costmap:
```yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  rolling_window: false
  track_unknown_space: true
  update_frequency: 5.0
  static_map: true

#  plugins:
#   - {name: static_layer, type: "costmap_2d::StaticLayer"}
#   - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
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


new local costmap:
```yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  rolling_window: true
  update_frequency: 2.0
  publish_frequency: 2.0
  static_map: false

  width: 3.0 # influences velocity if too small, the velocity will be also smaller !
  height: 3.0 # influences velocity if too small, the velocity will be also smaller !
  resolution: 0.05

#  plugins:
#    - {name: obstacle_layer, type: "costmap_2d::ObstacleLayer"}
#    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
```

common costmap params:
```yaml
footprint: [[0.2375, 0.1], [0.2375, -0.1], [-0.2375, -0.1], [-0.2375, 0.1]]
footprint_padding: 0.01

# Sensor Settings
obstacle_range: 6.0
raytrace_range: 6.0
inflation_radius: 0.15 # the surroundings of obstacle
lethal_cost_threshold: 100
map_type: costmap

# Robot Characteristics
min_obstacle_height: 0.1
max_obstacle_height: 2.0

# Obstacle Layer Settings
obstacle_layer:
  enabled: true
  combination_method: 1
  observation_sources: laser_scan
  laser_scan: {sensor_frame: laser, data_type: LaserScan, topic: scan, marking: true, clearing: true}

# Inflation Layer Settings
inflation_layer: # activate inflation layer (pink local map)
  enabled: true
```

base global planner:
```yaml
GlobalPlanner:
  allow_unknown: false  # Do not plan through unknown space.
  use_dijkstra: true    # Use Dijkstra's algorithm for pathfinding.
  use_quadratic: false  # Disable quadratic approximation (only needed for complex terrain).
  use_grid_path: false  # Generate smoother paths instead of grid-aligned.
  old_navfn_behavior: false  # Use new behaviors for smoother planning.
  tolerance: 0.2        # 0.0 No tolerance for goal; robot must reach it exactly.
  lethal_cost: 100      # Define obstacle threshold (use the same as your costmap).
  neutral_cost: 50      # Neutral cost for regular cells, if increased robot takes shortest path.
  cost_factor: 3.0      # Weight of costmap cells in path calculation.
  orientation_mode: 1   # Align path to preferred orientation (1 = along path direction). if 0 Robot ignores orientation (not suitable for Ackermann robots).
  default_tolerance: 0.5  # Allow slight deviation in path to make navigation feasible.
```

base_local_planner_params.yaml :
```yaml
TrajectoryPlannerROS:
  # Robot Configuration Parameters
  acc_lim_x: 2.5          # Maximum acceleration in the x direction (m/s^2)
  acc_lim_theta: 0.0      # No rotational acceleration since Ackermann robots can't rotate in place
  max_vel_x: 2.5          # Maximum linear velocity (m/s)
  min_vel_x: -1.5         # Allow backward movement
  max_vel_theta: 0.0      # No rotational velocity
  min_vel_theta: 0.0      # No rotational velocity
  min_in_place_vel_theta: 0.0  # No rotation in place
  holonomic_robot: false  # Ackermann robots are not holonomic

  # Trajectory Parameters
  sim_time: 2.0           # Time to forward-simulate trajectories (seconds)
  sim_granularity: 0.05   # Distance between points on a trajectory (meters)
  angular_sim_granularity: 0.05 # Ignored for Ackermann robots
  vx_samples: 20           # Number of samples for velocities in x
  vy_samples: 0           # Set to 0 for non-holonomic robots
  vtheta_samples: 1       # No theta samples for Ackermann robots

  # Goal Tolerances
  xy_goal_tolerance: 0.2  # Distance tolerance to the goal (meters)
  yaw_goal_tolerance: 0.1 # Yaw tolerance to the goal (radians)
  latch_xy_goal_tolerance: false  # Do not latch goal tolerance

  # Path Scoring
  path_distance_bias: 32.0  # Prefer paths that follow the global plan
  goal_distance_bias: 24.0  # Prefer paths that reach the goal
  occdist_scale: 0.01       # Scaling factor for obstacle cost

  # Forward Simulation Parameters
  heading_lookahead: 0.1    # was 0.3 Distance to look ahead in trajectory for heading alignment
  dwa: false                # Disable Dynamic Window Approach; use trajectory rollout
  meter_scoring: true       # Use scoring robust against costmap resolution changes

  # Obstacle Avoidance Parameters
  max_obstacle_distance: 2.0 # Maximum distance to consider obstacles
  inflation_radius: 0.4      # Robot’s inflation radius for costmap
  
  # Kinematic Constraints
  max_trans_vel: 1.0         # Maximum translational velocity (m/s)
  min_trans_vel: 0.05        # Minimum translational velocity (m/s)
  backup_vel: -0.1           # Backup velocity (negative for backward motion)

  # Additional Configuration
  yaw_goal_tolerance: 0.2    # Allow some yaw tolerance at the goal
  prune_plan: true           # Prune the global plan as the robot moves
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
```xml
<?xml version="1.0"?>

<launch>

<!-- Map server -->
  <node name="map_server" pkg="map_server" type="map_server" args="/home/anas/navicar_2/src/naviconfig/maps/my_map_now.yaml"/>

  <!-- Enable arduino connection -->
  <node name="rosserial_python" pkg="rosserial_python" type="serial_node.py">
    <param name="port" value="/dev/USB-ARDUINO"/>
    <param name="baud" value="57600"/>
  </node>

  <!-- Load the URDF -->
  <!-- <param name="robot_description" command="cat /home/anas/navicar_2/src/naviconfig/urdf/car_working.urdf"/> -->
  <!-- <param name="use_sim_time" value="false" /> -->

  <!-- Odometry simulation -->
  <node pkg="navicar_2_2" type="fake_odom_publisher" name="fake_odom_publisher" output="screen"> 
  <remap from="odom" to="raw_odom"/>
  </node>

  <!-- Static transform for laser -->
  <node pkg="tf" type="static_transform_publisher" name="base_to_laser" args="0 0 0 0 0 0 base_link laser 100"/>

<!-- EKF -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization" output="screen">
    <param name="use_sim_time" value="false"/>
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/ekf.yaml" command="load"/>
    <remap from="odometry/filtered" to="odom"/>
  </node>

  <!-- AMCL node -->
  <node name="amcl" pkg="amcl" type="amcl" output="screen">
    <!-- <param name="use_map_topic" value="true"/> -->
    <param name="odom_frame_id" value="odom"/>
    <param name="base_frame_id" value="base_link"/>
    <param name="global_frame_id" value="map"/>
    <param name="update_min_d" value="0.1"/>
    <param name="update_min_a" value="0.1"/>
    <param name="odom_model_type" value="diff"/>
    <param name="odom_alpha5" value="0.1"/>
<!--Lowering transform_tolerance value may lead to Expolration Error. Adjust this value according to the processor capacity -->
      <param name="transform_tolerance" value="0.5" /> 
      <param name="gui_publish_rate" value="10.0"/> 
      <param name="laser_max_beams" value="30"/>
      <param name="min_particles" value="500"/>
      <param name="max_particles" value="2000"/>
      <param name="kld_err" value="0.05"/>
      <param name="kld_z" value="0.99"/>
      <param name="odom_alpha1" value="0.2"/>
      <param name="odom_alpha2" value="0.2"/> 
      <param name="odom_alpha3" value="0.8"/>
      <param name="odom_alpha4" value="0.2"/>
      <param name="laser_z_hit" value="0.5"/>
      <param name="laser_z_short" value="0.05"/>
      <param name="laser_z_max" value="0.05"/>
      <param name="laser_z_rand" value="0.5"/>
      <param name="laser_sigma_hit" value="0.2"/>
      <param name="laser_lambda_short" value="0.1"/>
      <param name="laser_lambda_short" value="0.1"/>
      <param name="laser_model_type" value="likelihood_field"/>
      <param name="laser_likelihood_max_dist" value="2.0"/>
      <param name="update_min_d" value="0.1"/>
      <param name="update_min_a" value="0.2"/>
      <param name="resample_interval" value="2"/>
      <param name="recovery_alpha_slow" value="0.0"/>
      <param name="recovery_alpha_fast" value="0.0"/>
      <param name="ackermann_steering" value="true"/>
      <param name="wheelbase" value="0.254"/>
      <param name="track_width" value="0.167"/>
  </node>

    <!-- Move base # IMPORTANT: base_local_planner  base local planner can only handle holonomic/differential drive robots and assumes roughly circular shape. Tolerated for car-like robots-->

    <!-- Move Base Node -->
  <node pkg="move_base" type="move_base" name="move_base" output="screen">
    <!-- Move Base Core Parameters -->
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/move_base_2.yaml" command="load" />

    <!-- Base Local Planner Parameters -->
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/base_local_planner_2.yaml" command="load" />

    <!-- Global Planner Parameters -->
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/global_planner_2.yaml" command="load" />
    <!-- Costmap Configuration -->
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/commoncostmap3.yaml" command="load" ns="global_costmap" />
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/commoncostmap3.yaml" command="load" ns="local_costmap" />
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/local_cost_2.yaml" command="load" />
    <rosparam file="/home/anas/navicar_2/src/naviconfig/config/global_cost_2.yaml" command="load" />
    
  </node>

  <!-- RViz -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d /home/anas/navicar_2/src/naviconfig/rviz/my_config_2.rviz"/>
</launch>
```
