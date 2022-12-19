## 14/12/2022

<br />

# Report of the eighth session

In the eighth session of building our autonomous RC car, I came across a lot of problems and did modify a lot of things in the program :

- First off, I wired up the components of the car to the Arduino card, then I launched the program that I had already written last in the session. In the beginning, only the laser sensor worked with no problems but the servo and the motor were not moving at all, although in the last session it seemed to be working well. After checking the wires, I found that I two pins were inverted and one pin was connected to the positive side of the generator whlie it should not. Unfortunately, even after making sure the wiring was good, the servo and the motor still refuse to work. My supervisor checked the code and made some major changes to it. He added some basic functions to run each one of the car components. The laser worked fine, but the motor and th servo motor did respond to the new code lines. He then checks the wiring and found one **major flaw which was that the ground pins for the servo motor and the DC motor were not connected to the ground pin of the Arduino card**. This flaw will prevent the two parts from recognizing the code properly. Then, after running the code again, all the parts worked together.<br />
After tweaking the code a little, the car is now capable of detecting obstacles from the laser sensor readings and react to them accordingly. The new code is :<br />


`#include <Servo.h>`<br />
`//#include "CytronMotorDriver.h"`<br />
`#include <Wire.h>`<br />
`#include <VL53L0X.h>`<br />

`VL53L0X sensor;`<br />

`// Configure the motor driver.`<br />
`// CytronMD motor(PWM_DIR,4, 9);  // DIR = Pin 9, PMW = Pin 4. These are the pins that were inverted. Now I am not using this command line, it is replaced by another one bellow. I am still using the MD13S motor driver`<br />

`int Dir = 4;`<br />
`int PWM = 3;`<br />
`int pos = 0;    // variable to store the servo position`<br />
`Servo servo1; // Pin 5`<br />
`//int dist=0;`<br />


`//Delay alternative, still working on it to make it replace the "delay" function`<br />
`//unsigned long currentMillis = millis();`<br />
`//unsigned long previousMillis = 0;`<br />
`//const long interval=1000;`<br />


`void setup() {`<br />
  `// put your setup code here, to run once:`<br />

  `pinMode(PWM, OUTPUT);`<br />
  `pinMode(Dir, OUTPUT);`<br />
 
  `servo1.attach(5);`<br />
 
  `servo1.write(70); // turn right`<br />
  `delay(1000);`<br />
  `servo1.write(50); // wheels back to the center`<br />
  `delay(1000);`<br />
  `Serial.begin(9600);`<br />
  `Wire.begin();`<br />
  `sensor.init();`<br />
  `sensor.setTimeout(500);`<br />

  `sensor.startContinuous();`<br />

`}`<br />



`void loop() {`<br />

`/* Test: Works perfect for Motor, servo and laser sensor`<br />
  
`int dist=0;`<br />
`dist = sensor.readRangeContinuousMillimeters();`<br />
`Serial.println(dist);`<br />

`digitalWrite(Dir, HIGH); // Direction set to HIGH (forward in my case) (LOW = backwards), if the motor runs backwards, you may invert its wires that are connected to the motor driver`<br />
`analogWrite(PWM, 150); // Sets the speed of the motor (150). It can go from 0 to 255`<br />
`  servo1.write(70);`<br />
`delay(1000);`<br />
`analogWrite(PWM, 200);`<br />
`  servo1.write(90);`<br />
`delay(1000);`<br />
`analogWrite(PWM, 0);`<br />
`  servo1.write(70);`<br />
`delay(1000);`<br />
`//digitalWrite(Dir, HIGH);`<br />
`analogWrite(PWM, 150);`<br />
`  servo1.write(90);`<br />
`delay(1000);`<br />
`analogWrite(PWM, 200);`<br />
`  servo1.write(70);`<br />
`delay(1000);`<br />
`analogWrite(PWM, 0);`<br />
`  servo1.write(90);`<br />
`delay(1000);`<br />

`*/`<br />

`  //Sensor`<br />
  
`  int dist=0; // The distance between the sensor and the obstacle, if there was one, if not, 8160 as a max value`<br />
`  dist = sensor.readRangeContinuousMillimeters();`<br />
`  dist = dist - 30; // -30 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
`  Serial.print("Distance: ");`<br />
`  Serial.print(dist);`<br />
`  Serial.print("mm");`<br />
`  Serial.println();`<br />

`  //Motor`<br />

`  start();`<br />
`  watchout();`<br />
`  backup();`<br />
  
`}`<br />

`// Functions used above`<br />

`//void wait1s() { I am still working on this function to replace "delay"`<br />
`  //previousMillis = millis();`<br />
`  //while (currentMillis - previousMillis <= 2000) {`<br />
`    //currentMillis = millis();`<br />
`  //}`<br />
`//}`<br />


`void start() {`<br />
`  int dist = sensor.readRangeContinuousMillimeters();`<br />
`  if (dist >= 2000) {`<br />
`      servo1.write(50);`<br />
`      digitalWrite(Dir, HIGH);`<br />
`      analogWrite(PWM, 100);}`<br />
`      // if while is used instead of if, use : int dist = sensor.readRangeContinuousMillimeters(); inside the loop`<br />
`}`<br />

`void backup() {`<br />
`  int dist = sensor.readRangeContinuousMillimeters();`<br />
`  if (dist <= 200) {`<br />
`    //digitalWrite(Dir, HIGH);`<br />
`    analogWrite(PWM, 0);`<br />
`    servo1.write(90); // Move the servo to the right`<br />
`    delay(1000);`<br />
`    digitalWrite(Dir, LOW);`<br />
`    analogWrite(PWM, 150);`<br />
`    delay(2000);`<br />
`    servo1.write(10); // Move the servo to the left`<br />
`    digitalWrite(Dir, HIGH);`<br />
`    analogWrite(PWM, 0);`<br />
`    servo1.write(50);`<br />
`  }`<br />
`}`<br />

`void watchout() {`<br />
`  int dist = sensor.readRangeContinuousMillimeters();`<br />
`  if (dist <= 1000) {`<br />
`    digitalWrite(Dir, HIGH);`<br />
`    analogWrite(PWM, 80);`<br />
`    servo1.write(70); // Move the servo to the right`<br />
`    delay(1000); `<br />
`  }`<br />
  
`}`<br />

- After getting the car to work with this code, there are some little changes to make, like omitting the functions at the bottom and working with a harmonious function rather than executing each one at a time. The wiring of the car with the Arduino card is as shown below :

![20221214_174011](https://user-images.githubusercontent.com/115218309/207728804-a42f526d-ac04-42b8-9e38-ce9cac873bee.jpg)
![20221214_173958](https://user-images.githubusercontent.com/115218309/207728817-d79ed340-21a9-4806-9072-e70bbf530d5d.jpg)
![20221214_161045](https://user-images.githubusercontent.com/115218309/207728821-135fc9ca-3749-4be3-9f0a-89f4712dbcb7.jpg)
![20221214_174002](https://user-images.githubusercontent.com/115218309/207728825-6e75013c-e0b9-47d6-8cf2-9fab64e5d475.jpg)
![20221214_174006](https://user-images.githubusercontent.com/115218309/207728830-bbd56670-55ef-4b21-93ff-68dbdc44e088.jpg)

