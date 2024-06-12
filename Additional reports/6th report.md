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
