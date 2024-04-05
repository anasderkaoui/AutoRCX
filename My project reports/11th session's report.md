## 31/01/2023

<br />

# Report of the eleventh session

In this session, I have worked mainly on the hardware used for the car:<br />

- Since the inception of this project, I have been utilizing an Arduino UNO board to program the car's code. Now, I have an opportunity to experiment with a new board that was recently given to me by my supervisor. This new board is similar to the Arduino Mega but manufactured by a different company. Despite using the same coding language, I have been informed that this board boasts a significantly larger memory capacity and a greater number of ports, providing more room for expansion in case I decide to add additional components or modules to the car in the future.<br />

![image](https://user-images.githubusercontent.com/115218309/216465573-7b7a3c01-eaac-4261-bdbe-0b737dd5062f.png)

- The new Bluetooth module is an exciting addition to the project, offering the ability to communicate with the Arduino card through a Bluetooth terminal on a mobile phone. This opens up new possibilities for remote control and automation, such as the ability to send a destination to the car and have it autonomously navigate to that location. The module resembles a small, compact device, ready to bring new levels of convenience and efficiency to the project :<br />

![image](https://user-images.githubusercontent.com/115218309/216465799-c1329a98-a84b-4791-aeeb-576db652f4dc.png)
![image](https://user-images.githubusercontent.com/115218309/216465813-19108f45-c242-48d6-899b-46f9cc980004.png)

This Bluetooth module holds immense potential for various applications, and our project is not an exception. The versatility of this module enables it to be tailored to fit the specific needs of each application. In our case, we aim to utilize its capabilities to develop a platform that will allow us to send a location to the module, which in turn will translate it into appropriate commands to drive the car to its desired destination. The implementation of this will require us to either use existing platforms or create a new one from scratch, but the end result will be a car that can drive autonomously to a given location, thanks to this powerful Bluetooth module. With this technology at our disposal, we are excited about the endless possibilities that lie ahead for our project and how it can contribute to the advancement of the field.<br />

The Bluetooth module HC-05 has 6 pins for establishing the connection :<br />

VCC: power pin, typically connected to the Arduino's 5V pin<br />
GND: ground pin, typically connected to the Arduino's GND pin<br />
RX: receive pin, typically connected to the Arduino's transmit (TX) pin<br />
TX: transmit pin, typically connected to the Arduino's receive (RX) pin<br />
State: returns 1 when the module is connected<br />
Key or EN must be powered to enter the configuration mode and should not be connected to be in communication mode.<br />

The uniqueness of the HC-05 Bluetooth module is that it can be used in slave mode (in which case it is equivalent to a HC-06 module and is used in the same manner) or in master mode, meaning that it can autonomously connect to another Bluetooth module (HC-06, HC-05 in slave mode, or other) without external action.<br />

The Bluetooth module configuration can be interesting for checking the proper functioning of the module and for modifying its parameters, especially when you want to use the module in master mode. The module must be powered but not paired and the Key/EN pin must be powered at 3V when the module is turned on. When the module is in configuration mode, the LED will light up for two seconds every two seconds.<br />

Open the Arduino serial monitor and make sure the line ending option displays "Newline" and the baud rate is set to 9600.<br />

Note that the HC-05 Bluetooth module enters AT mode with a communication speed (baud rate) of 38400 bps.<br />

Configuration code :<br />

To manage communication with the HC-05 module, we use the SoftwareSerial.h library which allows us to create a serial port other than the one used by the USB port. The following code allows you to modify the HC-05 module's parameters (name, PIN code, communication speed (baud rate), etc.) and find information such as the firmware version number.<br />

```
#include <SoftwareSerial.h>
#define rxPin 2
#define txPin 3
#define baudrate 38400
String msg;
SoftwareSerial hc05(rxPin ,txPin);
void setup(){
 	pinMode(rxPin,INPUT);
 	pinMode(txPin,OUTPUT);

 	Serial.begin(9600);
 	Serial.println("ENTER AT Commands:");
 	hc05.begin(baudrate);
}
void loop(){
 			readSerialPort();
 			if(msg!="") hc05.println(msg);

 			if (hc05.available()>0){
 					Serial.write(hc05.read());
 			}
}
void readSerialPort(){
 	msg="";
 while (Serial.available()) {
 		delay(10);
 		if (Serial.available() >0) {
 				char c = Serial.read(); 	//gets one byte from serial buffer
 				msg += c; //makes the string readString
 		}
 }
}
```
To ensure good communication, make sure to select the correct baudrate in the serial monitor and "new line" (NL) in the communication settings.<br />

Configuration Commands :

Aside from being able to use it in master mode, the HC-05 Bluetooth module also allows you to know its saved parameters.<br />

Generally, if you type AT+<command>? in the serial monitor, you will get the value stored in the module (e.g. AT+PSWD? to know the module's PIN code). If you enter the line AT+<command>=<Param>, you will set the value of the module (e.g. AT+PWSD=0000 to change the PIN code to 0000).<br />

Here are some basic AT commands to know:<br />

- To test communication, type AT in the Arduino IDE's serial monitor. If everything is fine, the module should respond with OK.<br />
- To change the name of the module, type AT+NAME=<Param> module. The module should respond with OK. (E.g. To change the module name to BTM1, type AT+NAME=BTM1).<br />
- To change the module's PIN code, type AT+PSWD=<Param>. The module should respond with OK. (E.g. if you want to change the PIN to 0000, type AT+PSWD=0000).<br />
- AT+ROLE=<Param> to change the module's role as slave or master (E.g. to set the module as master, type AT+ROLE=1).<br />
- To change the module's communication speed (only if necessary), type AT+UART=<Param1>,<Param2>,<Param3> with Param1, 2 and 3 being the serial communication parameters: baud rate, stop bit, and parity bit respectively. (Default is 9600,0,0. E.g. if you want to change the baudrate to 115200, type AT+UART=115200,0,0).<br />
