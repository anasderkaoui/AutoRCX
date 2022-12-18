# Report of the first session

In this first session of the building our autonomous RC car, I proceeded to do the following things:

- We didn’t receive the gear that we will need yet, so I will be browsing the internet to find codes for as many components as possible that we will be using in our car.
I searched for Arduino codes online for the proximity sensor:
Arduino Code for Proximity Sensor (This code works with any analog sensor!):

`//collects data from an analog sensor`<br />
`int sensorpin = 0; // analog pin used to connect the sharp sensor`<br />
`int val = 0; // variable to store the values from sensor(initially zero)`<br />
`void setup()`<br />
`{`<br />
`  Serial.begin(9600); // starts the serial monitor`<br />
`}`<br />
`void loop()`<br />
`{`<br />
`  val = analogRead(sensorpin);       // reads the value of the sharp sensor`<br />
`  Serial.println(val);            // prints the value of the sensor to the serial monitor`<br />
`  delay(400);                    // wait for this much time before printing next value`<br />
`}`

- Finding out about a new motor, special brushed motor, that we can use in our RC car:
Our supervisor advised that I use a special motor for the project car rather than the brushless motors that I foresaw. His arguments were that this special motor, namely RUNNER SPECIAL 2 LRP ELECTRONIC, is especially designed to handle and run perfectly in RC cars. Moreover, this motor has the ability of going forward and backward unlike the brushless motor that is designed to go one way and in order to make it go backwards there must be some additional connections and an ESC on top of that, which will add more code lines and more weight to the car. The motor provided by the supervisor only needs an H-Bridge card. You can find out more about the motor in this link: [LRP website](https://www.lrp.cc/en/product/runner-special-2/)

![rapport1-1image](https://user-images.githubusercontent.com/115218309/201213191-78fa4ad5-41a8-401c-a5be-b9190a0f2c51.jpg)

- I looked up how to connect the motor and H-bridge with the Arduino card:
It is not the first time that I will be using an H-Bridge card, but it is my first time using it to control a brushed motor for our project car. I have used an H-bridge card previously to control a small brushless motor. The brushed motor our supervisor handed over to us draws 12A (128W) and functions in the range of 4,8V to 9,6V. The H-bridge reference is Cytron-MD10C R3, the card is bigger than the one that we were planning to use, but it performs better.

- Exploring a previous project found in the local lab and searching for any similarities with our project:
I found that the RC car has a servomotor similar to that we are going to use in our car.

![rapport1-2image](https://user-images.githubusercontent.com/115218309/201213300-821d694c-c67d-42c5-8031-5fe953c57520.jpg)

What is more is that the H-bridge card is the same as the one we have, and this will make it easy for us in a certain way to make connections between the Arduino card, the brushed motor and the H-bridge card.
Finally, I noticed that this car has some sensors and a camera, so this gave me an idea of some places where to put the proximity sensors in our project car, and what modifications I should do on the body or the frame of the car in order to put the camera where it should be.

![rapport1-3image](https://user-images.githubusercontent.com/115218309/201213333-28030943-7b07-4c54-bc84-32e8802691b9.jpg)
