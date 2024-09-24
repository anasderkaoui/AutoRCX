In this session I have reached this level in my project:

- The car is now running well the SLAM algorithm and can generate a map of the environment very confidently.
- The car has some additional components like some sensors and an additional power converter (buck) that are not in use for the moment and I don't see that they will be used in the future. Therefore, it would be good to get rid of some of them if not used and leave some space for other component.
- The Arduino card is always connected to the Jetson Nano and is responsible for controlling the steering and speed.
- The car can be controlled wirelessly with a PS4 controller, only that the buttons should be remapped and/or the instructions sent from the controller to the car are not very well implemented by the program. Last time I checked, car responds to only some buttons and the response is jerky. Have to improve the program. This will allow to move the car around for performing the SLAM. I can also have another program to control the car by keyboard.
- The final target which is autonomous navigation. Last time I checked, a long time ago, the algorithms implemented seem to be the right ones, but they are not fine tuned the car itself. So I have to create a package with every comprehensive algorithm and its utility and how it helps the car.

- As of today, the 24/09/2024, I will concentrate on the hardware problems since they will not take much time, and then I will move on the the wireless and autonomous parts.

- UPDATE:
- The converter has been removed !
- Now I am working on building a package to control the car by keyboard.
