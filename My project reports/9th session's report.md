## 16/12/2022

<br />

# Report of the ninth session

In this session, I have mainly worked on the program of the car :

- As always, I tested to see if the code was running so far so good before doing any changes to it. Then, I began apply some subtle modifications just to make it look more appealing to the average user. After that, I noticed something going on with the car shaft that is connected directly to the DC motor. The car, when triggered, was doing a sound of friction between two metals. When I saw the connection I found that the motor gear is scratching the surface of the frame's gear. I unscrewed the motor and then placed it safely in order to make good contact with the other gear. Here is an image from where that sound was coming from :

![Screenshot_20221216_221307_Gallery](https://user-images.githubusercontent.com/115218309/208196614-43c3eeee-64ac-4088-b5f6-d56cb3707fff.jpg)

- After figuring out the friction sound on the car, now comes the more complicated part for this session. The code is working great and all, but the car is getting the readings from only one laser sensor. For it to be more sophisticated and work properly, I added two more HC-SR04 ultrasonic sensors that will further help guide the car. Those sensors are not as precise as the laser sensor, but they will only be beneficial for when the car is near an obstacle, that way it will help avoid some objects of an unusual form. Moreover, I wanted to add a laser sensor to give the distance at the back of the car, but it is not as easy as I was thinking. I looked up on the internet and found an article but did not try the given code, beacuse I was busy writing the code for the ultrasonic sensors. After a lot of changes that I have done to the code, the car can now receive readings from the new sensors and can respond accordingly. I also want to point out one mistake that I have made while modifying the code but was not aware of. **The mistake was that I had defined two sensors but only one Trig pin and one Echo pin. Because of this subtle mistake the readings of the two sensors were the same since the output and the input are defined only once**. The final code is :

`#include <Servo.h>`<br />
`#include <Wire.h>`<br />
`#include <VL53L0X.h>`<br />

`// HC-SR04 Right Sensor`<br />
`#define echoPin1 12 // attach pin D12 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin1 11 //attach pin D11 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationRight; // variable for the duration of sound wave travel`<br />
`int SoundDistanceRight; // variable for the distance measurement`<br />

`// HC-SR04 Left Sensor`<br />
`#define echoPin2 7 // attach pin D7 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin2 6 //attach pin D6 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationLeft; // variable for the duration of sound wave travel`<br />
`int SoundDistanceLeft; // variable for the distance measurement`<br />


`VL53L0X sensor;`<br />
`//VL53L0X sensor2;`<br />

`int Dir = 4;`<br />
`int PWM = 3;`<br />

`int pos = 0;    // variable to store the servo position`<br />
`Servo servo1; // Pin 8`<br />


`void setup() {`<br />
  `// put your setup code here, to run once:`<br />

  `//Motor`<br />

  `pinMode(PWM, OUTPUT);`<br />
  `pinMode(Dir, OUTPUT);`<br />

  `// Servo`<br />

  `servo1.attach(5);`<br />
  `servo1.write(70); // turn right`<br />
  `delay(1000);`<br />
  `servo1.write(50); // wheels back to the center`<br />
  `delay(1000);`<br />

  `// Sensor 1`<br />

  `Serial.begin(9600); // Serial Communication is starting with 9600 of baudrate speed`<br />
  `Wire.begin();`<br />
  `sensor.init();`<br />
  `sensor.setTimeout(500);`<br />
  `sensor.startContinuous();`<br />

  `// Sensor 2`<br />

  `//Wire.begin();`<br />
  `//sensor2.init();`<br />
  `//sensor2.setTimeout(500);`<br />
  `//sensor2.startContinuous();`<br />

  `// HC-SR04 sensors`<br />

  `pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
  `pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT`<br />

  `pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
  `pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT`<br />

`}`<br />


`void loop() {`<br />

  `// Laser Sensor`<br />

  `int dist = 0;`<br />
  `dist = sensor.readRangeContinuousMillimeters();`<br />
  `dist = dist - 30; // -55 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
  `Serial.print("dist: ");`<br />
  `Serial.print(dist);`<br />
  `Serial.print("mm");`<br />
  `Serial.println();`<br />

  `// Laser Sensor 2`<br />

  `//int dist2 = 0;`<br />
  `//dist2 = sensor.readRangeContinuousMillimeters();`<br />
  `//dist2 = dist2 - 30; // -55 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
  `//Serial.print("dist2: ");`<br />
  `//Serial.print(dist2);`<br />
  `//Serial.print("mm");`<br />
  `//Serial.println();`<br />

  `//Motor`<br />

  `if (dist <= 150) {`<br />
    `//digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 0);`<br />
    `digitalWrite(Dir, LOW);`<br />
    `analogWrite(PWM, 100);`<br />
    `delay(500);`<br />
    `while (SoundSensorRight() <= 18) { // in centimeters`<br />
      `servo1.write(95); //Move the servo to the right`<br />
      `delay(1000);`<br />
      `digitalWrite(Dir, LOW);`<br />
      `analogWrite(PWM, 100);`<br />
      `Serial.print("Let's go left => ");`<br />
      `Serial.println();`<br />
    `}`<br />
    `while (SoundSensorLeft() <= 18) {`<br />
      `servo1.write(15); //Move the servo to the right, 15 is the minimum not 10`<br />
      `delay(1000);`<br />
      `digitalWrite(Dir, LOW);`<br />
      `analogWrite(PWM, 100);`<br />
      `Serial.print("Let's go right => ");`<br />
      `Serial.println();`<br />
    `}`<br />
  `}`<br />

  `if (SoundSensorRight() <= 25) {`<br />
    `digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 100);`<br />
    `servo1.write(30); //Move the servo to the right, 15 is the minimum not 10`<br />
    `delay(1000);`<br />
  `}`<br />
  `else if (SoundSensorLeft() <= 25) {`<br />
    `digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 100);`<br />
    `servo1.write(80); //Move the servo to the right, 15 is the minimum not 10`<br />
    `delay(1000);`<br />
  `}`<br />

  `//int dist = sensor.readRangeContinuousMillimeters();`<br />
  `else if (dist <= 1000) {`<br />
    `digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 100);`<br />
    `servo1.write(75);`<br />
    `delay(1000);`<br />
  `}`<br />

  `//int dist = sensor.readRangeContinuousMillimeters();`<br />
  `else if (dist >= 2000) {`<br />
    `servo1.write(55);`<br />
    `digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 150);`<br />
  `}`<br />

`}`<br />


`/*    if (dist2 >= 100) {  // This one is intended to work with the second laser sensor after wiring it up and setting it up with the Arduino card<br />`<br />
    `digitalWrite(Dir, LOW);`<br />
    `analogWrite(PWM, 100);`<br />
    `delay(1000);`<br />
    `servo1.write(10);`<br />
    `digitalWrite(Dir, HIGH);`<br />
    `analogWrite(PWM, 0);`<br />
    `//delay(1000); //wait1s();`<br />
    `}`<br />
    `else {`<br />
      `servo1.write(10);`<br />
      `analogWrite(PWM, 100);`<br />
      `delay(1000);`<br />
    `}`<br />
`*/`<br />

`int SoundSensorRight() {`<br />

  `// HC-SR04 Sensor`<br />

  `// Clears the trigPin condition`<br />
  `digitalWrite(trigPin1, LOW);`<br />
  `delayMicroseconds(2);`<br />
  `// Sets the trigPin HIGH (ACTIVE) for 10 microseconds`<br />
  `digitalWrite(trigPin1, HIGH);`<br />
  `delayMicroseconds(10);`<br />
  `digitalWrite(trigPin1, LOW);`<br />
  `// Reads the echoPin, returns the sound wave travel time in microseconds`<br />
  `durationRight = pulseIn(echoPin1, HIGH);`<br />
  `// Calculating the distance`<br />
  `SoundDistanceRight = durationRight * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)`<br />
  `// Displays the distance on the Serial Monitor`<br />
  `Serial.print("Right SoundDistance: ");`<br />
  `Serial.print(SoundDistanceRight);`<br />
  `Serial.println(" cm");`<br />
  `return SoundDistanceRight;`<br />
`}`<br />

`int SoundSensorLeft() {`<br />

  `// HC-SR04 Sensor`<br />

  `// Clears the trigPin condition`<br />
  `digitalWrite(trigPin2, LOW);`<br />
  `delayMicroseconds(2);`<br />
  `// Sets the trigPin HIGH (ACTIVE) for 10 microseconds`<br />
  `digitalWrite(trigPin2, HIGH);`<br />
  `delayMicroseconds(10);`<br />
  `digitalWrite(trigPin2, LOW);`<br />
  `// Reads the echoPin, returns the sound wave travel time in microseconds`<br />
  `durationLeft = pulseIn(echoPin2, HIGH);`<br />
  `// Calculating the distance`<br />
  `SoundDistanceLeft = durationLeft * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)`<br />
  `// Displays the distance on the Serial Monitor`<br />
  `Serial.print("Left SoundDistance: ");`<br />
  `Serial.print(SoundDistanceLeft);`<br />
  `Serial.println(" cm");`<br />
  `return SoundDistanceLeft;`<br />
`}`<br />

To beautify the code after copy-pasting it to the Arduino software, tap the combination "Ctrl+T".
I will keep you updated of when I get the second laser sensor working along with the other sensors.
