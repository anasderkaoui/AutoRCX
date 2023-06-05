In this session we will go through the basic stuff of ROS.<br >

First, here is a **[ROS cheatsheet](https://docs.google.com/document/d/1aHzvnf_MWg9O6PzCw1KsePNh27TZ9ND01cHMHroUVQE/edit?usp=sharing)** where you will find the most common commands in ROS.<br >

Example of publisher and subscriber in ROS :<br >

To create a project, we have to create some directories first. Create the following directory in your "home" directory: mkdir **catkin_ws** (You can name it as you like, but most programmers adapt these names). Before we go any further, we have to make sure that we have already **sourced the setup.bash file**. In order to do so, run this command:<br >
`source ~/catkin_ws/devel/setup.bash`<br >
After that, run theses cammands while in the catkin_ws directory:<br >
`catkin init` to initialize the workspace <br >
`mkdir src`<br >
`catkin build` build the necessary files for the ROS workspace <br >

Now we can create the publisher and subscriber. First let's create a package named "pub_sub": `catkin_create_pkg pub_sub` Then create the "scripts" folder where our scripts will be storred: `mkdir scripts` <br >
In this folder, run theses commands to create the publisher and subscriber: `touch publisher.py` then `touch subscriber.py` <br >
To make these files executable (change permissions), run this command: `chmod +rwx subscriber.py` then `chmod +rwx publisher.py` (You can also use +777 instead of +rwx. Replace + with - to remove permissions. Note that “r” is for read, “w” is for write, and “x” is for execute)<br >
After that was done, it is time to code ! Open the publisher first, using this command: `vim publisher.py` (You can also use nano if you want) and modify it as shown below. We will be using python2 **(ROS Melodic does not support python3)**:
```python
#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int64

# Node + Topic initialization
rospy.init_node("publisher_node")
pub = rospy.Publisher("topic", Int64, queue_size = 1)

# Looping
while not rospy.is_shutdown():
    pub.publish(1)
    rospy.sleep(1)
```
Open the subscriber, using this command: `vim subscriber.py` (You can also use nano if you want) and modify it as shown below:
```python
#!/usr/bin/env python2

import rospy
from std_msgs.msg import Int64

# Callback function
def callback(msg):
    print(msg)

# Node + Subscriber initialization
rospy.init_node("subscriber")
rospy.Subscriber("topic", Int64, callback) # callback is a function, can be named otherwise but it is a callback function

# Spinning
rospy.spin()
```
Now, let's run ROS to test the publisher and subscriber. Run these commands within the catkin_ws folder:
```bash
catkin build
source ~/catkin_ws/devel/setup.bash
roscore
```
Now open a new window within the terminal and type:
```bash
source ~/catkin_ws/devel/setup.bash
rosrun pub_sub publisher.py
```
To make sure publisher.py is working, open a new window and run this command that lists the topics that are running: `rostopic list` You should see the topic "/topic" running.
You can be more sure of that by typing: `rostopic echo /topic` <br >
And that is all for the publisher and subscriber !
