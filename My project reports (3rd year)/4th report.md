In this session I have reached this level in my project:

The car can be controlled wirelessly with a PS4 controller, it can steer and go back and forth well. However one problem is that, if a lot of instruction are sent, it takes time to do them and will do them entirely even with a big delay. Have to improve the program.

Another problem I have is that the DC motor can not move the car easily, sometimes it needs a push in order to move it. So I suppose that there is not enough voltage supply from the batteries that supply approximately 8 Volts.

As of today, the 25/10/2024, I will concentrate on seeing the root of the very slow movement problem and then I will try the autonomous navigation with the previous packages I had created last year.

UPDATE:

I am working on adding the a script that will subscribe to the published topics by the ps4 controller and translate them to the simulation.

I am also working on the SLAM part with a fusion of the robot simulation.

I have successfully tested the robot model in a real-time SLAM and I could clearly see the robot model move the car moves in real life ! Although I still have to adjust some parameters (the wheels don't steer and also I think all the wheel aren't turning, just translating the model). I also have adjusted the origin of the model with real life (the model is below the generated map)!

