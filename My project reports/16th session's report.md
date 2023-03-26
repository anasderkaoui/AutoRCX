## 21/03/2023

<br />

# Report of the sixteenth session

<br />

Driving an RC car autonomously can be a challenging task, especially when it comes to avoiding obstacles. While the car is designed to navigate around obstacles, there are still situations where it hits a wall, causing frustration and a need for improvement.<br />

In the past, I wrote a code that worked effectively, but as time passed, it needed updates to handle specific conditions. Unfortunately, after tweaking some of the "if" statements, the car started to behave erratically and would frequently crash into walls. It was a mystery that required thorough testing and investigation.<br />

Eventually, I discovered that the car's sensors had a significant limitation. The laser sensor, which was the primary sensor for obstacle detection, had a narrow field of recognition that only detected obstacles directly in front of it. On the other hand, the ultrasonic sensors had a more extensive detection range. It was clear that a wider range of detection was necessary for the car to navigate successfully.<br />

To address this limitation, I figured that adding another ultrasonic sensor above the laser sensor could complete the gamut of obstacle detection at the front of the RC car. Thus providing the car with the necessary information to avoid obstacles more effectively and prevent collisions.<br />

Additionally, as we are planning to add more components, and as I previously mentioned, we are considering upgrading to the MEGA2560 Strong board. However, when I attempted to connect it to the laser sensor, which required SDA and SCL connections, I encountered a challenge. Despite my efforts, I could not locate the required pins on the board, and my online search for the board's datasheet was fruitless.<br />

Fortunately, our supervisor came to our aid and suggested that we use the authentic Arduino Mega 2560 board as a replacement. This new board not only had the necessary SCL and SDA connections but also offered an abundance of additional pins that far surpassed those of the Arduino UNO R3 board we are currently using.<br />

![1009197d-cce2-4096-a7dc-b20ee3d09b38](https://user-images.githubusercontent.com/115218309/227792419-a274af88-cc95-4b47-aadf-92a05e4e2e07.jpg)
![Arduino-Mega-Pinout](https://user-images.githubusercontent.com/115218309/227792427-55d5459e-90d1-49db-892c-047c5bf31f85.jpg)

The upgrade to the Arduino Mega 2560 board has proven to be a wise decision as it has enhanced our ability to connect multiple components and expand our project's capabilities. Moreover, with its ample pins, we can now connect more sensors, actuators, and other components, allowing us to gather more data and achieve more significant results.<br />
