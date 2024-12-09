In this session I have reached this level in my project:

The car is fully able to navigate relying only on the battery that I have received which is great news!
I had to make some changes to the dwa planner yaml file so that it matches my robot's needs (DWA planner is adapted to the differential drive robots by defaults, so I had to change it because I have an ackerman robot, robot with wheel steering). I also made some changes to the launch file and most importantly to the Arduino code. It is now able to detect values varying from -1 to 1 for the steering!<br>

When the car moves forward in the real world, in the simulation it appeares to be backing up ! It is like if something is inverted in the simulation ! After some little research, I found that there might be something related to the tf tree (in URDF and/or launch file probably) that causes this problem!

As of today, the 09/12/2024:

-> I will try to fix the problem where the scans of the LiDAR don't match the LiDAR movements. <br>
-> I will create a publisher/subscriber that will subscribe to the "cmd_vel" topic and publish its values to the "joint_state" topic in order to see the robot movement on simulation! But that will be done only when I resolve the simualtion issue.

UPDATE:

I thought it was going to be pretty straightforward to make the car drive autonomously but I ran into some chalenges that I have to resolve:

1- The problem kinda persists with the LiDAR scans not matching real LiDAR movements. When tweeking the launch file, I found that this has to do with tf orientation. Although I found the parameters to moify, but still I don't get the right output when trying all possible combinations.

2- I found [this](https://robotics.stackexchange.com/questions/100364/lidar-sensing-backwards) and maybe it could help to resolve this issue. Remains to check other forums and community.

3- When the robot is moving, the LiDAR scans are also moving with it, as if the already saved map has no effect on th efact that the scans should be superimposed to the obstacles and stay fixed like that!

4- I also found that some URDF gets loaded even if I comment out that part, maybe that would also cause a problem. Also I **have to tweek the parameters in the working URDF and see if this fixes it!**

5- I still did not install the IMU. The new cables for the battery work well as well as the new batteries. **Optimal charging for batteries 11.1V to 11.55V. MAX 12.6V MIN 9.6V** To charge batteries, specify **5.0A as default**.
