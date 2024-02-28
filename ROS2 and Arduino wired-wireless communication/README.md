I will walk you through how to set a communication between ROS2 and Arduino. I used ROS2 Foxy with an Arduino Portenta H7, Arduino Nano RP2040 Connect and a Raspberry Pi Pico. Go to [this repo](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy) for more information.<br>

**Important Note**: This will only be compatible with few boards, among of which the Arduino Portenta H7, Arduino Nano RP2040 Connect and the Raspberry Pi Pico. For the full list of compatible devices, refer to the repository above.

Explanation:
In ROS there is a library that enables us to send and receive message from and to a microcontroller, which is **rosserial**. ROS2, on the other hand, can be equipped with a library that is still recent, but can enable us to do the same thing as on ROS. It is called **micro-ROS**, this tool will help us achieve what we can do on ROS, like communnicating an Arduino board to a ROS2 PC.

In this example we will be running the code for the ultrasonic sensors, we will be following a 2-step process. First we will install the required library, which is **micro-ros**, then we will move the micro ros library to Arduino libraries and upload the code to the board.

**There will be another example where we are going to publish IMU data in order to visualize MPU6050 on RVIZ2 using ROS 2 Foxy !**

## First step: Installing micro-ros

### Building

Create a ROS 2 workspace and build the package:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash # Source ROS2 distribution

mkdir uros_ws && cd uros_ws # Create a workspace and go to there

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

colcon build --symlink-install # Build your workspace, the argument added lets you modify your workspace without needing to rebuild after any changes each time in most cases

source install/local_setup.bash
```

Once the package is built, the firmware scripts are ready to run. But before that, we need to create and build the agent that will behave as a publisher and a subscriber at the same time. 

### Building micro-ROS-Agent

Using this package is possible to install a ready to use **micro-ROS-Agent**:

While being in your workspace, run the following commands (remember to build and source if not done previously):

```bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
``` 

Now you might run into an error related to "TinyXML2", here is the fix:<br>
- Go to the [TinyXML2 repo](https://github.com/leethomason/tinyxml2) and download the ZIP file under the green "Code" button.
- Extract the compressed file, rename it to `tinyxml2` and move it to the "Home" directory.
- Finally run the commands again and it should work properly.<br>

This is the last command that enables us to use micro-ros:<br>
`ros2 run micro_ros_agent micro_ros_agent [parameters]` in parameters, you could specify: `udp4 --port 8888` or `serial --dev /dev/ttyUSB0` depending on whether you want to connect using internet (udp4) or USB cable (/dev/ttyACM0 for example).
If you run this command you will see this:

![Image 1](image.png)

## Second step: Installing Arduino and the corresponding libraries

Now that micro-ros is set up, go ahead and download the [Arduino IDE](https://www.arduino.cc/en/software) that will help us upload sketches to the microcontroller.
After the download, make sure that the IDE is working fine by unzipping the file and running the Arduino application (arduino-ide). If you cannot run the app, open a terminal in the same folder and type `./arduino-ide`.

In Arduino, add the necesary libraries for your microcontroller. See [the suggested libraries](https://docs.arduino.cc/software/ide-v2/tutorials/ide-v2-board-manager/#mbed-os-nano) for each Arduino microcontroller. You can use other microcontrollers like the ESP32, STM32 or even a Raspberry Pi Pico.

You might need to install `pip` and `pyserial`:<br>
`sudo apt-get install pip` then `sudo pip install pyserial`

Now download [the micro-ros-arduino repository](https://github.com/micro-ROS/micro_ros_arduino.git). This repo is packed with micro-ros codes that, when uploaded to the microcontroller, will enable us to communicate with to read incoming information from the board. After the download is finished:
- Unzip the folder
- Go to "Home" and then go to (or create if not available yet) `Arduino/libraries`
- Paste the unzipped file and rename it to "micro_ros_arduino"
- Reload the Arduino IDE
- Now you should be able to see on the IDE, under "Examples", "micro_ros_arduino".
- Go ahead and upload any example (like "micro-ros_publisher") to your microcontroller.
If you cannot upload a code to the microcontroller, make sure that you gave **permission to the port** by running:
`sudo chmod a+rw /dev/ttyACM0`, your device might be on another port (/dev/ttyUSB0, /dev/ttyUSB1 ...), verify that on the Arduino IDE where you selected your board.

**If you are using a new Arduino Nano RP2040 Connect, you should run the "post_installation" file before uploading a sketch to it, or else it would display an error**:<br>
Open the terminal and run: 
- `cd ~/.arduino15/packages/arduino/hardware/mbed_nano/` then if you type `ls` you will see probably a folder or multiple folders named with versions, choose the latest version and go to it: `cd <version>`
- `sudo ./post_install.sh`

**If you are using an Arduino Portenta, make sure to [update the firmware and WIFI firmware](https://support.arduino.cc/hc/en-us/articles/4403365234322-Update-Wi-Fi-firmware-on-Portenta-H7-boards)**

After this, you are all good to go. Run the micro ros Agent: `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0` and you should see something like this:

![Image 2](image-1.png)

- Open another tab from the terminal
- Enter this: `source /opt/ros/$ROS_DISTRO/setup.bash  && export ROS_DOMAIN_ID=0 && ros2 topic list`
- Then you should see a topic named: "ultrasonics_publisher"
- Type: `ros2 topic echo /ultrasonics_publisher` and you should see the final result:

![Image 3](image-2.png)

[Arduino code for ultrasonic sensors](https://github.com/anasderkaoui/AutoRCX/blob/main/ROS2%20and%20Arduino%20wired-wireless%20communication/ultrasonics_code.ino) **using Wi-Fi/UDP (wireless communication)**, if you want to use the same code but through **wired communication** you should modify [this line](https://github.com/anasderkaoui/AutoRCX/blob/8624060a2ac2a25c2fc45ead49ba957ea84c4387/ROS2%20and%20Arduino%20wired-wireless%20communication/ultrasonics_code.ino#L82) to be:
`set_microros_transports()`
