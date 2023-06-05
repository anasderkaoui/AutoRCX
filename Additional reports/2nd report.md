In this session we will go through the basic stuff of ROS.

First, here is a **[ROS cheatsheet](https://docs.google.com/document/d/1aHzvnf_MWg9O6PzCw1KsePNh27TZ9ND01cHMHroUVQE/edit?usp=sharing)** where you will find the most common commands in ROS.

Example of publisher and subscriber in ROS :<br >

To create a project, we have to create some directories first. Create the following directory in your "home" directory: mkdir **catkin_ws** (You can name it as you like, but most programmers adapt these names). Before we go any further, we have to make sure that we have already **sourced the setup.bash file**. In order to do so, run this command:<br >
`source ~/catkin_ws/devel/setup.bash`<br >
After that, run theses cammands while in the catkin_ws directory:<br >
`catkin init` to initialize the workspace <br >
`mkdir src`<br >
`catkin build` build the necessary files for the ROS workspace <br >

Now we can create the publisher and subscriber. First let's create a package named "pub_sub": `catkin_create_pkg pub_sub` Then create the "scripts" folder where our scripts will be storred: `mkdir scripts` <br >
In this folder, run theses commands to create the publisher and subscriber: `touch publisher.py` then `touch subscriber.py`.
To make these files executable, run this command: `chmod +rwx subscriber.py` then `chmod +rwx publisher.py` (You can also use +777 instead of +rwx)
After that is done, it is time to code ! Open the publisher first, using this command: `vim publisher.py` (You can also use nano if you want) and modify it as shown below. We will be using Python2 **(ROS Melodic does not support python3)**:
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
