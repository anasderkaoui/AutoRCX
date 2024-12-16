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

1- I have found some new parametrs that I can fiddle with. Tried to put 3.142 on odom tf broadcaster on all 3 slots but nothing changed. I know this problem is non existent when I do the mapping procedure, so I figured out I wil just take a look at the parameters in the hector mapping folder and might there be a solution to this problem!!
After a lot of param modifications the problem still persists!! See laser tf broadcaster params and hector mapping params!

2- I found [this](https://robotics.stackexchange.com/questions/100364/lidar-sensing-backwards) and [this also](https://robotics.stackexchange.com/questions/21009/ros-laser-scan-rotates-with-the-robot-in-rviz) maybe it could help to resolve this issue. Remains to check other forums and community.

3- When the robot is moving, the LiDAR scans are also moving with it, as if the already saved map has no effect on th efact that the scans should be superimposed to the obstacles and stay fixed like that!

4-

5- I still did not install the IMU. The new cables for the battery work well as well as the new batteries. **Optimal charging for batteries 11.1V to 11.55V. MAX 12.6V MIN 9.6V** To charge batteries, specify **5.0A as default**.
