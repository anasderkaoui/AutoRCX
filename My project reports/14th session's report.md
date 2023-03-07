## 07/03/2023

<br />

# Report of the fourteenth session

In today's session, my focus was on enhancing the autonomous car's ability to travel to a destination of our choice. To accomplish this task, I conducted extensive research on the internet to identify the necessary modules that must be added to the Arduino card. The goal was to enable the car to determine its current location and find the best route to the next destination.<br />

After several hours of researching, I concluded that we would require new modules to achieve this goal. The first module we need is the Bluetooth module, which we already possess but still needs to be set up with a mobile app. The second module is the GPS module, which will help us send location data, here is an example of the code that it will be running :

`#include <SoftwareSerial.h>`<br />
`#include <TinyGPS.h>`<br />

`SoftwareSerial gpsSerial(2, 3); // RX, TX`<br />
`TinyGPS gps;`<br />

`void setup()`<br />
`{`<br />
`  Serial.begin(9600);`<br />
`  gpsSerial.begin(9600);`<br />
`}`<br />

`void loop()`<br />
`{`<br />
`  while (gpsSerial.available())`<br />
`  {`<br />
`    char c = gpsSerial.read();`<br />
`    if (gps.encode(c))`<br />
`    {`<br />
`      double lat, lng;`<br />
`      gps.f_get_position(&lat, &lng);`<br />
`      Serial.print("Latitude: ");`<br />
`      Serial.print(lat, 6);`<br />
`      Serial.print(", Longitude: ");`<br />
`      Serial.println(lng, 6);`<br />
`    }`<br />
`  }`<br />
`}`<br />

![module-gps-gy-neo6mv2-compatible-arduino-raspberry-stm32](https://user-images.githubusercontent.com/115218309/223479475-d11ec511-64bd-478a-95ee-b81c910f750a.jpg)

And the third module is the Compass, which will allow the car to determine which direction to go. Here is a version of its code :

`#include <Wire.h>`<br />
`#include <Adafruit_Sensor.h>`<br />
`#include <Adafruit_HMC5883L.h>`<br />

`Adafruit_HMC5883L mag = Adafruit_HMC5883L();`<br />

`void setup()`<br />
`{`<br />
`  Serial.begin(9600);`<br />
`  Wire.begin();`<br />
`  mag.begin();`<br />
`}`<br />

`void loop()`<br />
`{`<br />
`  sensors_event_t event;`<br />
`  mag.getEvent(&event);`<br />
`  float heading = atan2(event.magnetic.y, event.magnetic.x);`<br />
`  if (heading < 0)`<br />
`    heading += 2 * PI;`<br />
`  if (heading > 2 * PI)`<br />
`    heading -= 2 * PI;`<br />
`  float heading_degrees = heading * 180 / M_PI;`<br />
`  Serial.print("Heading: ");`<br />
`  Serial.println(heading_degrees);`<br />
`  delay(500);`<br />
`}`<br />

![compassard](https://user-images.githubusercontent.com/115218309/223479533-492458da-cc76-49af-9fd3-c516798cf24f.jpg)

Our next step is to order the GPS and Compass modules and figure out how to set them up, and most importantly where to place them on the autonomous RC car. However, we encountered a problem since the Arduino Uno card we are currently using has only a few free pins available. Most of these pins are already in use by other components, such as the five ultrasonic sensors and servo motor.<br />

To resolve this issue, we have decided to switch to the Arduino Mega card that our supervisor provided us with.
![MegaStrong](https://user-images.githubusercontent.com/115218309/223478625-47e283d6-22af-4257-a6cb-d487596b7ef0.jpeg)
The Mega card has many more pins available and can handle more components than the Uno card. This change will require a new shield or box for the card and the modules, but we believe it is worth the effort.<br />

In conclusion, today's session was productive as we identified the necessary components needed to enhance the autonomous car's functionality. We also learned that it is essential to choose the appropriate Arduino card that can accommodate the modules needed to achieve our goals.<br />
