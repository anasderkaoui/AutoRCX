In this session I have reached this level in my project:

The car can be controlled wirelessly with a PS4 controller, it can steer and go back and forth well. However one problem is that, if a lot of instruction are sent, it takes time to do them and will do them entirely even with a big delay. Have to improve the program.

Another problem I have is that the DC motor can not move the car easily, sometimes it needs a push in order to move it. So I suppose that there is not enough voltage supply from the batteries that supply approximately 8 Volts.

As of today, the 25/10/2024, I will concentrate on seeing the root of the very slow movement problem and then I will try the autonomous navigation with the previous packages I had created last year.

UPDATE:

I have figured out the voltage supply problem. The battery cells I have right now do not provide enough power and most importantly amperage for the car components (jetson and dc motor especially) to work properlly. I will have to order a bigger battery with bigger C's and higher voltage (probably 11V).

I also received a first version of the IMU MPU6500 and will try to hook it up to the card and download its necessary packages.

