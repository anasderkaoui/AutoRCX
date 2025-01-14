In this session I have reached this level in my project:

The car is fully able to navigate relying only on the battery that I have received which is great news!
I had to make some changes to the dwa planner yaml file so that it matches my robot's needs (DWA planner is adapted to the differential drive robots by defaults, so I had to change it because I have an ackerman robot, robot with wheel steering). I also made some changes to the launch file and most importantly to the Arduino code. It is now able to detect values varying from -1 to 1 for the steering!<br>

When the car moves forward in the real world, in the simulation it appeares to be backing up ! It is like if something is inverted in the simulation ! After some little research, I found that there might be something related to the tf tree (in URDF and/or launch file probably) that causes this problem!

As of today, the 16/12/2024:

-> I will try to fix the problem where the scans of the LiDAR don't match the LiDAR movements. <br>
-> I will create a publisher/subscriber that will subscribe to the "cmd_vel" topic and publish its values to the "joint_state" topic in order to see the robot movement on simulation! But that will be done only when I resolve the simualtion issue.

UPDATE:

**base_link** or **base_footprint**

radians to degrees: 3.142 == 180.01, 1.571 == 90.01

I thought it was going to be pretty straightforward to make the car drive autonomously but I ran into some chalenges that I have to resolve:

1- I have finally found the problem that was causing the LiDAR scans to move with the robot in the simulation! **NOT HAVING AN ODOMETRY SOURCE IS THE PROBLEM**.
So in order to fix that, keeping in mind that I don't have an odometer/position sensor, I simulated odometry with a script(link to it) that updates the car's movement based on "cmd_vel" updates. And it is olso adapted to the car's dimentions and type (ackermann robot)! After simulating the odometry, the LiDAR scans seem to stay in place while the robot is moving which is **big great NEWS**. Now the LiDAR scans are also well put and oriented with the car (not inversed as previously without odometry)!

2- I ran into another problem that I know the cause of it. The LiDAR scans turn when the car turns. This is due to the fact that there is no IMU, hence no knowledge of real heading. Even if odometry is present, there maybe slippery or harsh surfaces that cause this behaviour. So next step is to add IMU and Sensor fuse it with odom (using robot_localization pkg) to get best estimation of pose possible. 

3- 
