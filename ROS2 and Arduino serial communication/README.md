I will walk you through how to set a communication between ROS2 and Arduino. I used ROS2 Foxy with an Arduino Portenta H7, Arduino Nano RP2040 Connect and a Raspberry Pi Pico. Go to [this repo](https://github.com/micro-ROS/micro_ros_arduino/tree/foxy) for more information.<br>

**Important Note**: This will only be compatible with few boards, among of which the Arduino Portenta H7, Arduino Nano RP2040 Connect and the Raspberry Pi Pico. For the full list of compatible devices, refer to the repository above.

Explanation:
In ROS there is a library that enables us to send and receive message from and to a microcontroller, which is **rosserial**. ROS2, on the other hand, can be equipped with a library that is still recent, but can enable us to do the same thing as on ROS. It is called **micro-ROS**, this tool will help us achieve what we can do on ROS, like communnicating an Arduino board to a ROS2 PC.

Code:
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
