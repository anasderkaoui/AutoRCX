## 25/11/2022

<br />

# Report of the fifth session

In this session, the car is becoming to come together. During this session :

- Our supervisor brought us two different DC motors to choose between. One was of high rpm and the other had a lower rpm. I ended up choosing the one with the higher rpm (approximately 14000) because it can be mounted on the chassis of the car and can handle heavy loads. However, there was a problem mounting the DC motor. The shaft was so short that we were not sure if the gear, that will be attached to it, will stick firmly both to the shaft and to the gear of the car frame. Therefore, the engineer, working with our supervisor, suggested to change the screws that were holding the motor to the mounting bracket and replace them with other ones that won't be protruding very much. While changing the screws, since the new ones that I will be using are longer, I learned that in order to shorten the length of a screw, it is of much importance to screw a nut to it first. This will help reform the thread of the end of the screw when unscrewing the nut. What is more is that, when placed where it needs to go, the DC motor is blocked by a piece of the car frame. The motor's shaft is still millimeters apart from the gear of the frame. The engineer took action and filed, with a drill, the part that was holding the motor from reaching the car gear. Ultimately, the motor is now well placed on the car body, we drilled some holes in the frame where the screws that will hold the motor in place will go.<br />

image

- I tried to actuate the servo motor that we placed the last session on the car, but ran into some issues. The servo's turning radius is 0 to 180 degrees, but when in the car it should not reach the extreme values because it is beyond what the car can handle. So I had two options, either change the servo's position which I tried but did not get the targeted results, or change the servo's program to only reach a certain value below the max. Therefore, I am going to choose the second option since the first one did not work out very well. The code Arduino that I modified habve now is :<br />


`// Controlling Speed`

`// Include the Servo library`<br />
`#include <Servo.h> `

`// Declare the Servo pin`<br />
`int servoPin = 3;`<br />
`// Create a servo object`<br />
`Servo Servo1;`

<br />

`void setup() {`<br />

`  // We need to attach the servo to the used pin number`<br />
`  Servo1.attach(servoPin);`<br />

`}`

<br />

`// The loop routine runs over and over again forever.`

`void loop(){`<br />

`  //I can change the values later if the servo is putting a lot of load on the wheels.`
`  // Make servo go to 30 degrees`<br />
`  Servo1.write(30);`<br />
`  delay(1000);`<br />
`  // Make servo go to 90 degrees`<br />
`  Servo1.write(90);`<br />
`  delay(1000);`<br />
`  // Make servo go to 150 degrees`<br />
`  Servo1.write(150);`<br />
`  delay(1000);`<br />

`}`
<br />

I did not get the chance to run this code on the servo but I am sure it will solve the problem.

- I found the right program for the <span style="color: green"> CYTRON MD13S DC motor driver </span> to run the DC motor. I tried it and it works very well. The Arduino program is :<br />

`#include "CytronMotorDriver.h"`

`// Configure the motor driver.`<br />
`CytronMD motor(PWM_DIR, 3, 4);  // PWM = Pin 3, DIR = Pin 4.`

`// The setup routine runs once when you press reset.`<br />

`void setup() {`
  
`}`

`// The loop routine runs over and over again forever.`<br />

`void loop() {`

`  motor.setSpeed(128);  // Run forward at 50% speed.`
`  delay(1000);`
  
`  motor.setSpeed(255);  // Run forward at full speed.`
`  delay(1000);`

`  motor.setSpeed(0);    // Stop.`
`  delay(1000);`

`  motor.setSpeed(-128);  // Run backward at 50% speed.`
`  delay(1000);`
  
`  motor.setSpeed(-255);  // Run backward at full speed.`
`  delay(1000);`

`  motor.setSpeed(0);    // Stop.`
`  delay(1000);`

`}`

image.arduinohookup