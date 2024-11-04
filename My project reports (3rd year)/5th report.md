In this session I have reached this level in my project:

The car can be controlled wirelessly with a PS4 controller, it can steer and go back and forth well. However one problem is that, if a lot of instruction are sent, it takes time to do them and will do them entirely even with a big delay. This is very probably dur to the code size that is imported to the Arduino R3 board. I remember when uploading the code that I had a warning telling me that the code uses almost 90% of the card's capacity which can lead to some problems

As of today, the 04/11/2024, I will concentrate on getting the MPU9052 IMU to work with ROS, then I will try the autonomous navigation with the previous packages that I had created last year.

UPDATE:

I learnt that pip is used to install dependencies of python1, pip2 for python 2 and pip3 for python 3. And each time I run a command, it overwrites the last one !

The library I want to use for the MPU9260 uses python3, however ROS melodic uses python2, so I have to find a driver that is a little outdated for the IMU to work!

After looking on the internet, I found that there is a compatible ros melodic driver for the BNO055 IMU. So I think I amgoing to go with this sensor and let go of the current one.
