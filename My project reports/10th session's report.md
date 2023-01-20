## 16/01/2023

<br />

# Report of the tenth session

During this session, I have worked on the program of the car, and I have also modified some parts of the car :

- Firstly, I tested the code because I made some big changes since the last time I have tested it. It tested great and was ready to be put on the Arduino card. After I uploaded it, I plugged the car to the generator and saw the results. They were evry good, the car is able to detect any hinderance in front of it or at the sides and always tries to avoid it. It does so either by steering away and changing directions or by doing a u turn in accordance with the objects that are blocking the way. The code is :

`/* Replacing delay with its alternative is not a good idea for every function */`<br />

`#include <Servo.h>`<br />
`//#include "CytronMotorDriver.h"`<br />
`#include <Wire.h>`<br />
`#include <VL53L0X.h>`<br />

`// HC-SR04 Front Right Sensor`<br />
`#define echoPin1 8 // attach pin D12 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin1 9 //attach pin D11 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationFR; // variable for the duration of sound wave travel`<br />
`int SoundDistanceFR; // variable for the distance measurement`<br />

`// HC-SR04 Front Left Sensor`<br />
`#define echoPin2 7 // attach pin D7 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin2 6 //attach pin D6 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationFL; // variable for the duration of sound wave travel`<br />
`int SoundDistanceFL; // variable for the distance measurement`<br />

`// HC-SR04 Rear Right Sensor`<br />
`#define echoPin3 12 // attach pin D7 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin3 11 //attach pin D6 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationRR; // variable for the duration of sound wave travel`<br />
`int SoundDistanceRR; // variable for the distance measurement`<br />

`// HC-SR04 Rear Left Sensor`<br />
`#define echoPin4 2 // attach pin D7 Arduino to pin Echo of HC-SR04`<br />
`#define trigPin4 5 //attach pin D6 Arduino to pin Trig of HC-SR04`<br />

`// defines variables`<br />
`long durationRL; // variable for the duration of sound wave travel`<br />
`int SoundDistanceRL; // variable for the distance measurement`<br />

`VL53L0X sensor;`<br />
`//VL53L0X sensor2;`<br />

`// Configure the motor driver.`<br />
`//CytronMD motor(PWM_DIR,4, 9);  // DIR = Pin 9, PMW = Pin 4.`<br />
`int Dir = 4;`<br />
`int PWM = 3;`<br />

`int pos = 0;    // variable to store the servo position`<br />
`Servo servo1; // Pin 10, was 5`<br />

`//Delay alternative`<br />
`unsigned long previousMillis = 0;`<br />
`unsigned long interval = 1200;  // interval at which to perform the task (milliseconds)`<br />
`unsigned long LongInterval = 2200;  // interval at which to perform the task (milliseconds)`<br />


`void setup() {`<br />

`  // put your setup code here, to run once:`<br />

`  //Motor`<br />
`  pinMode(PWM, OUTPUT);`<br />
`  pinMode(Dir, OUTPUT);`<br />

`  // Servo`<br />
`  servo1.attach(10); // was 5`<br />
`  servo1.write(70); // turn right`<br />
`  checkInterval();`<br />
`  servo1.write(55); // wheels back to the center`<br />
`  checkInterval();`<br />

`  // Sensor 1`<br />
`  Serial.begin(9600); // Serial Communication is starting with 9600 of baudrate speed`<br />
`  Wire.begin();`<br />
`  sensor.init();`<br />
`  sensor.setTimeout(500);`<br />
`  sensor.startContinuous();`<br />

`  // Sensor 2`<br />
`  //Wire.begin();`<br />
`  //sensor2.init();`<br />
`  //sensor2.setTimeout(500);`<br />
`  //sensor2.startContinuous();`<br />

`  // HC-SR04 sensors`<br />
`  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
`  pinMode(echoPin1, INPUT); // Sets the echoPin as an INPUT`<br />
`  pinMode(trigPin2, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
`  pinMode(echoPin2, INPUT); // Sets the echoPin as an INPUT`<br />
`  pinMode(trigPin3, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
`  pinMode(echoPin3, INPUT); // Sets the echoPin as an INPUT`<br />
`  pinMode(trigPin4, OUTPUT); // Sets the trigPin as an OUTPUT`<br />
`  pinMode(echoPin4, INPUT); // Sets the echoPin as an INPUT`<br />
`}`<br />

`void loop() {`<br />

`  // Laser Sensor`<br />
`  int dist = 0;`<br />
`  dist = sensor.readRangeContinuousMillimeters();`<br />
`  dist = dist - 30; // -30 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
`  Serial.print("Front distance: ");`<br />
`  Serial.print(dist);`<br />
`  Serial.print("mm");`<br />
`  Serial.println();`<br />

`  // Laser Sensor 2`<br />
`  //int dist2 = 0;`<br />
`  //dist2 = sensor.readRangeContinuousMillimeters();`<br />
`  //dist2 = dist2 - 30; // -30 is to compensate for error. Change or set it to zero to make it work for your sensor`<br />
`  //Serial.print("dist2: ");`<br />
`  //Serial.print(dist2);`<br />
`  //Serial.print("mm");`<br />
`  //Serial.println();`<br />

`  //Motor`<br />
`  //int dist = sensor.readRangeContinuousMillimeters();`<br />

`  if (dist <= 450) {`<br />

`    //if (SoundSensorFR() == SoundSensorFL()) {`<br />
`    servo1.write(55);`<br />
`    analogWrite(PWM, 0);`<br />
`    /*digitalWrite(Dir, LOW);`<br />
`      analogWrite(PWM, 57);*/`<br />
`    checkInterval();`<br />

`    if (SoundSensorRR() >= SoundSensorRL()) {`<br />
`      servo1.write(80);`<br />
`      analogWrite(PWM, 0);`<br />
`      digitalWrite(Dir, LOW);`<br />
`      analogWrite(PWM, 57);`<br />
`      checkInterval();`<br />
`    }`<br />

`    else if (SoundSensorRL() >= SoundSensorRR()) {`<br />
`      servo1.write(30);`<br />
`      analogWrite(PWM, 0);`<br />
`      digitalWrite(Dir, LOW);`<br />
`      analogWrite(PWM, 57);`<br />
`      checkInterval();`<br />
`    }`<br />

`    //}`<br />

`    if (SoundSensorFR() <= SoundSensorFL()) { // was else if  &  15 instead of SoundSensorFL()`<br />

`      if (SoundSensorRR() <= 10) {`<br />
`        servo1.write(55);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 0); // 52 if it was straight with the obstacle aside`<br />
`      }`<br />

`      else if (SoundSensorRL() >= 20) {`<br />
`        servo1.write(55);`<br />
`        analogWrite(PWM, 0);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`        servo1.write(15); // only if FR has become again free`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`
`        checkInterval();`<br />
`      }`<br />

`      else {`<br />

`        servo1.write(95); // Move the servo to the left`<br />
`        analogWrite(PWM, 0);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`      }`<br />

`    }`<br />

`    else if (SoundSensorFL() <= SoundSensorFR()) {`<br />

`      if (SoundSensorRR() <= 10) {`<br />
`        servo1.write(55);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 0); // 52 if it was straight with the obstacle aside`<br />
`      }`<br />

`      else if (SoundSensorRL() >= 20) {`<br />
`        servo1.write(55);`<br />
`        analogWrite(PWM, 0);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`        servo1.write(95); // only if FL has become again free`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`      }`<br />

`      else {`<br />
`        servo1.write`
`        checkInterval();`<br />
`      }`<br />

`      else {`<br />
`        servo1.write(35); // Move the servo to the right`<br />
`        analogWrite(PWM, 0);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`      }`<br />

`    }`<br />

`  }`<br />

`  else {`<br />
`    servo1.write(55);`<br />
`    digitalWrite(Dir, LOW);`<br />
`    analogWrite(PWM, 0);`<br />
`    checkInterval();`<br />
`    digitalWrite(Dir, HIGH);`<br />
`    analogWrite(PWM, 100);`<br />
`  }`<br />

`}`<br />

`// HC-SR04 Front Right Sensor`<br />
`int SoundSensorFR() {`<br />
`  digitalWrite(trigPin1, LOW);`<br />
`  delayMicroseconds(2);`<br />
`  digitalWrite(trigPin1, HIGH);`<br />
`  delayMicroseconds(10);`<br />
`  digitalWrite(trigPin1, LOW);`<br />
`  durationFR = pulseIn(echoPin1, HIGH);`<br />
`  SoundDistanceFR = durationFR * 0.034 / 2;`<br />
`  return SoundDistanceFR;`<br />
`}`<br />

`// HC-SR04 Front Left Sensor`<br />
`int SoundSensorFL() {`<br />
`  digitalWrite(trigPin2, LOW);`<br />
`  delayMicroseconds(2);`<br />
`  digitalWrite(trigPin2, HIGH);`<br />
`  delayMicroseconds(10);`<br />
`  digitalWrite(trigPin2, LOW);`<br />
`  durationFL = pulseIn(echoPin2, HIGH);`<br />
`  SoundDistanceFL = durationFL * 0.034 / 2;`<br />
`  return SoundDistanceFL;`<br />
`}`<br />

`// HC-SR04 Rear Right Sensor`<br />
`int SoundSensorRR() {`<br />
`  digitalWrite(trigPin3, LOW);`<br />
`  delayMicroseconds(2);`<br />
`  digitalWrite(trigPin3, HIGH);`<br />
`  delayMicroseconds(10);`<br />
`  digitalWrite(trigPin3, LOW);`<br />
`  durationRR = pulseIn(echoPin3, HIGH);`<br />
`  SoundDistanceRR = durationRR * 0.034 / 2;`<br />
`  return SoundDistanceRR;`<br />
`}`<br />

`// HC-SR04 Rear Left Sensor`<br />
`int SoundSensorRL() {`<br />
`  digitalWrite(trigPin4, LOW);`<br />
`  delayMicroseconds(2);`<br />
`  digitalWrite(trigPin4, HIGH);`<br />
`  digitalWrite(trigPin4, LOW);`<br />
`  durationRL = pulseIn(echoPin4, HIGH);`<br />
`  SoundDistanceRL = durationRL * 0.034 / 2;`<br />
`  return SoundDistanceRL;`<br />
`}`<br />

`void checkInterval() {`<br />
`  unsigned long currentMillis = millis();`<br />
`  if (currentMillis - previousMillis >= interval) {`<br />
`    // save the last time you blinked the LED`<br />
`    previousMillis = currentMillis;`<br />
`  }`<br />
`}`<br />

### Note that the function "checkInterval()" is not working properly. It does its job by replacing the "delay()" function but in the case of the Autonomous RC car it makes it hard for the car to take decisions properly. The car stutters when facing a front obstacle !
### If you want to use the code, please replace "checkInterval()" or "checkLongInterval()" with "delay(500)" and "delay(1500)" accordingly.

- After several trials, the car was doing a decent job of avoiding collisions. I then made some changes to the car itself. The big white breadboard was taking up so much blank space so my supervisor advised me to remove it and replace it with little boards that don't take much big space.

![20230120_160550](https://user-images.githubusercontent.com/115218309/213737177-89eb291f-843e-4e87-9832-d51e9e7ed638.jpg)

After that has been done, one other big problem was the wiring. As you can see on the very first page of the repository, they are all over the car and there is a big chance that something is going to short circuit and smoke. So that is what I am working on right now, and I have been told that all of the modules should go on the car frame and the Arduino card should go on the top so the wires won't be as tangled and matted. Lastly, this session I created the first version of a little carrier that will hold the back and front sensors together. Unfortunately, this was my first try and some pieces were not rigid enough to hold the sensors, and that was because I did not leave a border big enough to carry the weight.

![image](https://user-images.githubusercontent.com/115218309/213737645-0af44dd2-3f4c-4b15-8818-90a7495f534d.png)

![image](https://user-images.githubusercontent.com/115218309/213737743-4ce752b0-b17c-4f22-af54-d7adf9f146d7.png)

I have done some modifications on the carrier and will 3D print it next time in the same material, PETG, because it is well known for its rigidity.

![image](https://user-images.githubusercontent.com/115218309/213737879-5b9cbaf6-0b5d-4616-b9ec-abf93dac00f9.png)

![image](https://user-images.githubusercontent.com/115218309/213737930-791ea6b3-265a-4941-a1b4-89c308472628.png)

The little piece is the one that will hold the sensor, it will be screwed to it. And the longer piece will be put on the frame of the car to hold both sensors together.
