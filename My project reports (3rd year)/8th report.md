In this session I have reached this level in my project:

The car is fully able to navigate relying only on the battery tha I have received which is great news!
The car is able to navigate autonomously if I might say! There is a code that I should modify in order to  for the car to receive the complete commands!

As of today, the 28/11/2024:

-> I will try to make the car navigate autonomously.
-> I will create a publisher/subscriber that will subscribe to the "cmd_vel" topic and publish its values to the "joint_state" topic in order to see the robot movement on simulation!

UPDATE:

One other thing to fix is to add the tf tree of the robot's simulation in the launch file somewhere "I think" in order to get rid of the error that shows missing tf links!
