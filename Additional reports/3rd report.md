In this session we will be talking about launch files.

A launch file allows us to run many nodes simultanuously, without typing an individual ros command for each node. (Refer to the previous session to be up to speed with what will be done in this session)<br >
In order to do that, create a folder in the directory where the nodes reside: `mkdir launch` <br >
Then let's create and modify the launch file:<br >
`touch start.launch`<br >
`vim start.launch`<br >
```xml
<launch>
  <node name="publisher_node" type = "publisher.py" pkg = "pub_sub" output="screen"/>
  <node name="subscriber_node" type = "subscriber.py" pkg = "pub_sub" output="screen"/>
</launch>
```
Ultimately, let's launch the publisher and subscriber (After running roscore in a separate window):
`roslaunch pub_sub start.launch`

Little note: RQT is a ROS visualization tool that lets you see the architecture of the active projects. Run it (after starting roscore) by typing: `rqt`
