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

`  //Motor`<br />

`  if (dist <= 450) {`<br />

`    servo1.write(55);`<br />
`    analogWrite(PWM, 0);`<br />
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
`        checkInterval();`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        Serial.print("Let's go left => ");`<br />
`        Serial.println();`<br />
`        checkLongInterval();`<br />
`        if (SoundSensorFL() >= 10) {`<br />
`          servo1.write(15);`<br />
`          digitalWrite(Dir, HIGH);`<br />
`          analogWrite(PWM, 57);`<br />
`          checkInterval();`<br />
`        }`<br />
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
`        servo1.write(15);`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        checkInterval();`<br />
`      }`<br />

`      else {`<br />
`        servo1.write(15); // Move the servo to the right, 15 is the minimum not 10`<br />
`        checkInterval();`<br />
`        digitalWrite(Dir, LOW);`<br />
`        analogWrite(PWM, 57);`<br />
`        Serial.print("Let's go right => ");`<br />
`        Serial.println();`<br />
`        checkLongInterval();`<br />
`        if (SoundSensorFR() >= 10) {`<br />
`         servo1.write(95);`<br />
`         digitalWrite(Dir, HIGH);`<br />
`         analogWrite(PWM, 57);`<br />
`         checkInterval();`<br />
`       }`<br />
`     }`<br />
`   }`<br />
`   checkInterval();`<br />
` }`<br />

`if (dist >= 450 && dist <= 1500) {`<br />

`   servo1.write(55);`<br />
`   digitalWrite(Dir, HIGH);`<br />
`   analogWrite(PWM, 57);`<br />
`   checkInterval();`<br />

`   while (SoundSensorFL() >= 10 && SoundSensorFL() <= 20) { //general choice 100cm   //was if, but juttering`<br />
`     servo1.write(80);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`   }`<br />

`   if (SoundSensorFL() <= 10) {`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 0);`<br />
`     servo1.write(30);`<br />
`     checkInterval();`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`     servo1.write(80);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`   }`<br />

`   while (SoundSensorFR() >= 10 && SoundSensorFR() <= 20) { //general choice 100cm   //was if`<br />
`     servo1.write(30);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`   }`<br />

`   if (SoundSensorFR() <= 10) {`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 0);`<br />
`     servo1.write(80);`<br />
`     checkInterval();`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`     servo1.write(30);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`   }`<br />

` }`<br />

` else if (dist >= 1500) {`<br />

`   servo1.write(55);`<br />
`   digitalWrite(Dir, HIGH);`<br />
`   analogWrite(PWM, 57);`<br />
`   // if while is used instead of if, use : int dist = sensor.readRangeContinuousMillimeters();`<br />
`   while (SoundSensorFL() >= 10 && SoundSensorFL() <= 20) { //general choice 100cm   //was if`<br />
`     servo1.write(80);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`   }`<br />

`   if (SoundSensorFL() <= 10) {`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 0);`<br />
`     servo1.write(30);`<br />
`     checkInterval();`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`     servo1.write(80);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`   }`<br />

`   while (SoundSensorFR() >= 10 && SoundSensorFR() <= 20) { //general choice 100cm   //was if`<br />
`     servo1.write(30);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`   }`<br />

`   if (SoundSensorFR() <= 10) {`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 0);`<br />
`     servo1.write(80);`<br />
`     checkInterval();`<br />
`     digitalWrite(Dir, LOW);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`     servo1.write(30);`<br />
`     digitalWrite(Dir, HIGH);`<br />
`     analogWrite(PWM, 57);`<br />
`     checkInterval();`<br />
`   }`<br />
` }`<br />
`}`<br />

`//Functions called in the code`<br />

`// HC-SR04 Front Right Sensor`<br />

`int SoundSensorFR() {`<br />

` // Clears the trigPin condition`<br />
` digitalWrite(trigPin1, LOW);`<br />
` delayMicroseconds(2);`<br />
` // Sets the trigPin HIGH (ACTIVE) for 10 microseconds`<br />
` digitalWrite(trigPin1, HIGH);`<br />
` delayMicroseconds(10);`<br />
` digitalWrite(trigPin1, LOW);`<br />
` // Reads the echoPin, returns the sound wave travel time in microseconds`<br />
` durationFR = pulseIn(echoPin1, HIGH);`<br />
` // Calculating the distance`<br />
` SoundDistanceFR = durationFR * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)`<br />
` // Displays the distance on the Serial Monitor`<br />
` Serial.print("Front Right SoundDistance: ");`<br />
` Serial.print(SoundDistanceFR);`<br />
` Serial.println(" cm");`<br />
` return SoundDistanceFR;`<br />
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

`void checkLongInterval() {`<br />
` unsigned long currentMillis = millis();`<br />
` if (currentMillis - previousMillis >= LongInterval) {`<br />
`   // save the last time the task was performed`<br />
`   previousMillis = currentMillis;`<br />
` }`<br />
`}`<br />
