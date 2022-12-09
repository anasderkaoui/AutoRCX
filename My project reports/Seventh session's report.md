## 09/12/2022

<br />

# Report of the seventh session

In the seventh session of building our autonomous car, there's been a remarkable progress :

- I looked for the code of the "VL53L0/1XV2" sensor on the web, because last time it was working but using "Serial.begin(115200)", and this parameter hinders the use of other components like the servo motor. The code that I was looking for uses a data rate of 9600 that matches the servo's baud and also the the sensor's baud. The code is :

`/* The range readings are in units of mm. */`<br />

`#include <Wire.h>`<br />
`#include <VL53L0X.h>`<br />

`VL53L0X sensor;`<br />

`void setup()`<br />
`{`<br />
`  Serial.begin(9600);`<br />
`  Wire.begin();`<br />

`  sensor.init();`<br />
`  sensor.setTimeout(500);`<br />

`  // Start continuous back-to-back mode (take readings as fast as possible).  To use continuous timed mode instead, provide a desired inter-measurement period in ms (e.g. sensor.startContinuous(100)).`<br />

`  sensor.startContinuous();`<br />

`}`<br />

`void loop()`<br />
`{`<br />
`  int distance = sensor.readRangeContinuousMillimeters();`<br />
`  distance = distance - 55; // -55 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
`  Serial.print("Distance: ");`<br />
`  Serial.print(distance);`<br />
`  Serial.print("mm");`<br />
`  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }`<br />

`  Serial.println();`<br />
`}`<br />

Link to the code: [Source code for VL523L0X Distance measurement module for Arduino](https://robojax.com/learn/arduino/?vid=robojax-VL53L0X-polou)

- Using the code above and the other codes to run the servo motor and the AC motor, I succeeded at running the three of them simultaneously without any interference from one on the other. Howerver, I had to look for a new component that converts a higher voltage to a lower voltage. The motor's voltage input range is wider than the servo's, the motor should be having at least 7.4V going in for the motor driver to start working, thus turning the AC motor on. Except that the servo motor's voltage input range is of 4.8V up to 6.8V. This is the main reason why we need a voltage converter. In order to convert the 7.4V voltage output from the generator to 6.8V, our supervisor handed out a little pcb card that is capable of converting high voltages to low voltages.

![Converter](https://user-images.githubusercontent.com/115218309/206791566-10c8ce14-aca8-442a-bb3b-092e2f6b7754.jpg)

- After writing a new code, the car seems to work, but not very well. Some parts of the code are skipped or maybe not working. But the main target today was to get the laser sensor, the servo motor and the AC motor to work together. Ultimately, the code will need some tweaks and modifications to enable the AC motor and the servo motor to respond to the sensor's output. The code is :

`#include <Servo.h>`<br />
`#include "CytronMotorDriver.h"`<br />
`#include <Wire.h>`<br />
`#include <VL53L0X.h>`<br />

`VL53L0X sensor;`<br />

`// Configure the motor driver.`<br />
`CytronMD motor(PWM_DIR, 9, 4);  // DIR = Pin 9, PMW = Pin 4.`<br />

`int pos = 0;    // variable to store the servo position`<br />
`Servo servo1;`<br />


`void setup() {`<br />
`  // put your setup code here, to run once:`<br />
  
`  servo1.attach(8);`<br />
`  servo1.write(50);`<br />
`  motor.setSpeed(0);`<br />
`  Serial.begin(9600);`<br />
`  Wire.begin();`<br />

`  sensor.init();`<br />

`  // Start continuous back-to-back mode (take readings as fast as possible).  To use continuous timed mode instead, provide a desired inter-measurement period in ms (e.g. sensor.startContinuous(100)).`<br />
`  sensor.startContinuous();`<br />

`  /*for (pos = 50; pos <= 100; pos += 5) { // goes from 0 degrees to 180 degrees`<br />
`    // in steps of 1 degree`<br />
`    servo1.write(pos);              // tell servo to go to position in variable 'pos'`<br />
`    delay(15);                       // waits 15ms for the servo to reach the position`<br />
`  }`<br />
`  for (pos = 100; pos >= 50; pos -= 5) { // goes from 180 degrees to 0 degrees`<br />
`    servo1.write(pos);              // tell servo to go to position in variable 'pos'`<br />
`    delay(15);                       // waits 15ms for the servo to reach the position`<br />
`  }*/`<br />

`  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); } //sensor.setTimeout(500);`<br />
  
`}`<br />

`void loop() {`<br />

`//Sensor`<br />

`  int distance = sensor.readRangeContinuousMillimeters();`<br />
`  //distance = distance - 55;// -55 is to compensate for error. Change or set it to zero to make it work for your sensor. I personally did not feel the urge to use it the first time.`<br />
  
`//Motor`<br />
  
`  motor.setSpeed(100); // This line doesn't seem to work.`<br />
`  if (distance <= 800){ // This line (the whole "for" loop) worked some few times.`<br />
`      motor.setSpeed(100);`<br />
`      servo1.write(70); //Move the servo (90 degrees)`<br />
`      delay(1500);`<br />
`      servo1.write(10);}`<br />

`  if (distance <= 200){ // This line (the whole "for" loop) always worked.`<br />
`      motor.setSpeed(0);    // Stop.`<br />
`      delay(1000);`<br />
`      servo1.write(70); //Move the servo (90 degrees)`<br />
`      delay(1500);`<br />
`      motor.setSpeed(-128);`<br />
`      delay(1000);`<br />
`      servo1.write(10);`<br />
`      motor.setSpeed(128);`<br />
`      servo1.write(50);}`<br />

`//Servo`<br />

`  Serial.print("Distance: "); // These final commands always worked.`<br />
`  Serial.print(distance);`<br />
`  Serial.print("mm");`<br />
`  Serial.println();`<br />
  
`}`<br />

One thing I would like to add is that at the beginning, the sensor gives a range greater than 800 (or 200) many times on the serial monitor. But whenever the sensor is outputting a distance lower than 200 or 800 the values are then given very slowly on the serial monitor ( there is an output every 2s compared to milliseconds at the beginning) or even stop showing. Seeing that I have used the "delay" function that impedes the code for the amount of time given to it, I am not surprised that the code is not working properly. However, I am concerned with the fact that it stops working sometimes. Nevertheless, I will dig deep into this problem and try to solve it as soon as possible.

![CAR1](https://user-images.githubusercontent.com/115218309/206791720-765171c3-e2ca-4933-bd3d-7f8856e86760.jpg)

![CAR2](https://user-images.githubusercontent.com/115218309/206791728-26c89ecb-176b-475d-ad6b-f32cb7f9d160.jpg)

![CAR3](https://user-images.githubusercontent.com/115218309/206791731-2d0d3174-db96-4be6-8a87-6fc334a4129c.jpg)
