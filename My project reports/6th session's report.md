## 06/12/2022

<br />

# Report of the sixth session

In the sixth session of the building the autonomous RC car, I did but very little progress :

- I screwed the DC motor down in place but did not test it to see if it is still making a strong connection with the gear on the car.
Also, I may need to file some of the screws' head, that are holding the motor in place, because it is sticking out of the car frame.

![motoscrewed](https://user-images.githubusercontent.com/115218309/206027528-4e4c291b-1fce-40b4-8cf1-aef315636698.jpg)

- As promised from the last session, I found the code that will suit our servo motor. It is modified in order to keep a good connection with the steering arm. Because if it turned the wheels all the way to the extremes, the steering arm may break. There are two versions of the code that I had modified. The first one is :

`// Program for moving servo motor`<br />

`void loop()`<br />
`{`<br />
`val = 50; // 50 is center, 0 is left, 100 is right, I try to not reach the extremes`<br />
`myservo.write(val); `<br />
`delay(1500);`<br />
`val = 10; `<br />
`myservo.write(val); `<br />
`delay(1500);`<br />
`val = 50;`<br />
`myservo.write(val); `<br />
`delay(1500);`<br />
`val = 100;`<br />
`myservo.write(val); `<br />
`delay(1500);`<br />
`}`<br />

- You may wonder why I have two versions of the code that controls the servo motor. The answer is that I have tried to combine using the laser sensor, the servo motor and the DC motor to see if the car will stop if the laser is measuring a short distance, indicating an incoming obstruction. It turns out to be that the servo and the DC motor are not responding to the code from the Arduino card, and only the laser is working properly. When I asked my supervisor about that, he told that it could be the interference of the sensor on the two other parts. So he suggested to use another timer for the servo motor and giving it another try. The second code is :

`// Program for moving servo motor, only this time using the repository ServoTimer2`<br />

`#include"ServoTimer2.h"`<br />

`ServoTimer2 servo1;`<br />

`void setup() `<br />
`{`<br />
`servo1.attach(9);`<br />
`}`<br />
`void loop()`<br />
`{`<br />
`servo1.write(1155);  //  (1155) center`<br />
`delay(1000);`<br />
`servo1.write(1625);  // right pulse width for 90 degree`<br />
`delay(1000);`<br />
`servo1.write(1155);  //  (1155) center`<br />
`delay(1000);`<br />
`servo1.write(825);  //  left max pulse width for around 180 degree`<br />
`delay(1000);`<br />
`}`<br />

I had to tweak this version of the code to work properly with the servo motor, just like I did with the first one. When I tried to combine using the three parts together I had the same problem, only this time the servo does one rotation and doesn't move again. My supervisor then told me that a line in my code must be the problem. The line "Serial.begin(115200)" is very probably why the other parts refuse to work. He then suggests to revoke that line whenever I need to power on any other part of the car. Here is the old version of my code, before doing the necessary changes :

`#include <ServoTimer2.h> //Reading servo motor library`<br />
`#include "Adafruit_VL53L0X.h"`<br />
`#include "CytronMotorDriver.h"`<br />

`Adafruit_VL53L0X lox = Adafruit_VL53L0X();`<br />

`// Configure the motor driver.`<br />
`CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.`<br />


`ServoTimer2 myservo; //Create an object for the servo`<br />
`int val; //Variable for storing servo angle`<br />


`void setup() {`<br />

  `motor.setSpeed(100);  // Run forward at 50% speed.`<br />


  `Serial.begin(115200);  // THE PROBLEMATIC LINE `<br />

  `// wait until serial port opens for native USB devices`<br />
  `while (! Serial) {`<br />
   ` delay(1);`<br />
  `}`<br />
  
  `Serial.println("Adafruit VL53L0X test");`<br />
  `if (!lox.begin()) {`<br />
   ` Serial.println(F("Failed to boot VL53L0X"));`<br />
    `while(1);`<br />
  `}`<br />
  `// power `<br />
  `Serial.println(F("VL53L0X API Simple Ranging example\n\n")); `<br />
`}`<br />

`void loop() {`<br />

  
  `myservo.attach(9); //Set digital pin 9 as the command pin for determining the servo angle`<br />

  `myservo.write(1155);  //  (1155) center `<br />

  `myservo.write(1625);  // right pulse width for 90 degree`<br />

  `myservo.write(1155);  //  (1155) center `<br />

  `myservo.write(825);  //  left max pulse width for around 180 degree`<br />
  
  `VL53L0X_RangingMeasurementData_t measure;`<br />
    
  `Serial.print("Reading a measurement... ");`<br />
  `lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!`<br />

  `if (measure.RangeStatus != 4) {  // phase failures have incorrect data`<br />
    `Serial.print("Distance (mm): ");`<br />
    `Serial.println(measure.RangeMilliMeter);`<br />
    `}`<br />
  `else {`<br />
    `Serial.println(" out of range ");}`<br />

    
  `delay(100);`<br />
`}`<br />

There is still a lot to change in this code to enable the car to drive and avoid collisions.

![car](https://user-images.githubusercontent.com/115218309/206027583-8baa60d8-5277-4e6e-b6d8-a6e3e57bfc02.jpg)
