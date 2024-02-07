
// Updated code, it calculates average of last 15 values

// For the raspberry pi pico : VBUS is the micro-USB input voltage, connected to micro-USB port pin 1. This is nominally 5 V (or 0 V if the USB is not connected or not powered).
// VSYS is the main system input voltage, which can vary in the allowed range 1.8 V to 5.5 V, and which is used by the on-board SMPS (switch mode power supply) to generate the 3.3 V for the RP2040 and its GPIO.

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <cctype>

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Adjust pins depending on your setup
int triggerPins[] = { 2, 4, 6 };
int echoPins[] = { 3, 5, 7 };

unsigned long durations[3];

int standbydist = 0;
int finalDist = 0;

#define LED_PIN 13

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t* timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

bool isASCII(char c) {
  return (c >= 32 && c <= 126);
}
bool isAlNum(char c) {
  return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

void removeSpecialCharacters(char* str) {
  for (int i = 0; str[i] != '\0'; ++i) {
    if (!isASCII(str[i]) && isAlNum(str[i])) {  //!isalnum(str[i]) && !isspace(str[i])
      // If the character is not alphanumeric or a space, remove it
      for (int j = i; str[j] != '\0'; ++j) {
        str[j] = str[j + 1];
      }
      --i;
    }
  }
}

void setup() {
  set_microros_wifi_transports("SSID", "PASSWORD", "IP Adress of your PC", 8888); // Fill in your WIFI credentials. Port is generally set to 8888

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);

  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "ultrasonics", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "ultrasonics_publisher"));

  // create timer,
  const unsigned int timer_timeout = 700;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  for (int i = 0; i < 3; ++i) {
    pinMode(triggerPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT_PULLDOWN);
  }

  msg.data.data = " ";
}

void loop() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);

  delay(100);

  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  char terminal[100] = " ";

  for (int i = 0; i < 3; ++i) {

    digitalWrite(triggerPins[i], LOW);
    delayMicroseconds(10);
    digitalWrite(triggerPins[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPins[i], LOW);
    durations[i] = pulseIn(echoPins[i], HIGH, 40000UL);

    // Distance in centimeters
    double distanceCm = (double)durations[i] / 2.0 * 0.0343;

    for (int i = 0; i < 14; i++) {
      standbydist += distanceCm;
    }

    finalDist = standbydist / 15;  // Calculate average of 15 last values

    // Convert duration to string
    char durationString[10] = " ";

    // Show distance in numbers
    char innumbers[10] = " ";
    itoa(distanceCm, innumbers, 10);

    if (distanceCm >= 1 && distanceCm <= 30) { strcat(durationString, "RED"); }  // Danger zone, dangerously close to obstacle, should stop
    else if (distanceCm > 30 && distanceCm <= 70) {
      strcat(durationString, "ORANGE");
    }                                                                                    // Warning, robot very close to obstacle, should avoid collision or stop
    else if (distanceCm > 70 && distanceCm <= 100) { strcat(durationString, "YELLOW"); }  // Robot getting closer to an obstacle and should consider changing directions
    else {
      strcat(durationString, "GREEN");
    }  // Robot can navigate freely

    char direction[10] = " ";
    if (i == 0) {
      strcat(direction, "Right");
    } else if (i == 1) {
      strcat(direction, "Front");
    } else {
      strcat(direction, "Left");
    }  // é is a special caracter

    // Concatenate the phrase with the duration data
    char sensorData[50];
    sprintf(sensorData, "Distance %s is %i cm, %s\n", direction, finalDist, durationString);
    //char completeMessage[20] = "Distance " + String(i+1) + " is ";

    strcat(terminal, sensorData);

    // Print to serial for debugging
    Serial.println(sensorData);
    standbydist = 0;  // Reset the average to eliminate offset
  }
  removeSpecialCharacters(terminal);
  msg.data.data = terminal;
  standbydist = 0;  // Reset the average
}