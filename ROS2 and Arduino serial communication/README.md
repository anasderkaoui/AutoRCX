I will walk you through how to set a communication between ROS2 and Arduino. I used ROS2 Foxy with an Arduino Portenta H7, Arduino Nano RP2040 Connect and a Raspberry Pi Pico. Go to [this repo](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy) for more information.<br>

**Important Note**: This will only be compatible with few boards, among of which the Arduino Portenta H7, Arduino Nano RP2040 Connect and the Raspberry Pi Pico. For the full list of compatible devices, refer to the repository above.

Explanation:
In ROS there is a library that enables us to send and receive message from and to a microcontroller, which is **rosserial**. ROS2, on the other hand, can be equipped with a library that is still recent, but can enable us to do the same thing as on ROS. It is called **micro-ROS**, this tool will help us achieve what we can do on ROS, like communnicating an Arduino board to a ROS2 PC.

In this example we will be running the code for the ultrasonic sensors, we will be following a 2-step process. First we will install the required library, which is **micro-ros**, then we will move the micro ros library to Arduino libraries and upload the code to the board.

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

Now you might run into an error related "TinyXML2", here is the fix:<br>
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


Arduino code for ultrasonic sensors:
```C
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

int myTriggerPin = D1;  // Trigger on RangeFinder
int myEchoPin = D0;     // Echo on Rangefinder  
unsigned long myDuration;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  pinMode(myTriggerPin, OUTPUT);
  pinMode(myEchoPin, INPUT_PULLDOWN);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  digitalWrite(myTriggerPin, LOW);
  delayMicroseconds(10); 
  digitalWrite(myTriggerPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(myTriggerPin, LOW);
  myDuration = pulseIn(myEchoPin, HIGH, 40000UL);
  Serial.println("Duration us: "+ String(myDuration)+" mm");
  delay(200);
  if (myDuration > 2000    ){       // raw data from 200 to 16000                                          
                                        // where  2000 raw = ~35cm,  4000 raw = ~80cm                                    
       digitalWrite(LEDR, LOW);    // LEDB Blue LED on if far 
    } else { 
        digitalWrite(LEDR, HIGH);   // LEDB Blue LED off if close or nothing
    }

}


void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data = myDuration;
  }
}
```
