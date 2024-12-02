In this session I have reached this level in my project:

The car is fully able to navigate relying only on the battery tha I have received which is great news!
The car is able to navigate autonomously if I might say! There is a code that I should modify in order to  for the car to receive the complete commands!

As of today, the 28/11/2024:

-> I will try to make the car navigate autonomously. <br>
-> I will create a publisher/subscriber that will subscribe to the "cmd_vel" topic and publish its values to the "joint_state" topic in order to see the robot movement on simulation!

UPDATE:

One other thing to fix is to add the tf tree of the robot's simulation in the launch file somewhere "I think" in order to get rid of the error that shows missing tf links!
I had to make some changes to the dwa planner yaml file so that it matches my robot's needs (DWA planner is adapted to the differential drive robots by defaults, so I had to change it because I have an ackerman robot, robot with wheel steering). I also made some changes to the launch file and most importantly to the Arduino code. It is now able to detect values varying from -1 to 1 for the steering!

I thought it was going to be pretty straightforward to make the car drive autonomously but I ran into some chalenges that I have to resolve:

1- When the car moves forward in the real world, in the simulation it appeares to be backing up ! It is like if something is inverted in the simulation ! After some little research, I found that there might be something related to the tf tree (in URDF and/or launch file probably) that causes this problem!

2- When I set a goal plan and when the car almost reaches it, it keeps going in that same direction and therefore gets further away from the destination. The order keeps on being published!

3- When the robot is moving, the LiDAR scans are also moving with it, as if the already saved map has no effect on th efact that the scans should be superimposed to the obstacles!

4- I have received the new IMU but I did not set it yet.
