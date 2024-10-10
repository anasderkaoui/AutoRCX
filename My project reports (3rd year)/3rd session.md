In this session I have reached this level in my project:

The car has some additional components like some sensors and an additional power converter (buck) that are not in use for the moment. I have already removed the power converter but not the sensors, maybe I will use them afterwards.

The car can be controlled wirelessly with a PS4 controller, it can steer and go back and forth well. Hawever one problem is that, if a lot of instruction are sent, it takes time to do them and will do them entirely even with a big delay. Have to improve the program.

The final target which is autonomous navigation. Last time I checked, a long time ago, the algorithms implemented seem to be the right ones, but they are not finetuned to the car itself. So I have to create a package with every comprehensive algorithm and its utility and how it helps the car.

As of today, the 10/10/2024, I will concentrate on creating a simulation of the car itself so as to show it in RVIZ.

I succeded at getting a first look at the car visualization on RVIZ. The lidar position is also fixed !

UPDATE:

I am working on adding the a script that will subscribe to the published topics by the ps4 controller and translate them to the simulation.

I am also working on the SLAM part with a fusion of the robot simulation.

I have successfully tested the robot model in a real-time SLAM and I could clearly see the robot model move the car moves in real life ! Although I still have to adjust some parameters (the wheels don't steer and also I think all the wheel aren't turning, just translating the model). I also have adjust the origin of the model with real life (the model is below the generated map)!

