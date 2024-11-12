In this session I have reached this level in my project:

The car is fully able to navigate relying only on the battery tha I have received which is great news!
I have looked into the SLAM particle sampling and it does not have anything to do with the issue that I have whcih is map overlapping in SLAM. The particle sampling is a method similar to Monte Carlo Localization. It tries to guess the robot's initial localization, but then Adaptive Monte Carlo Localization takes over.

As of today, the 12/11/2024, I will try to generate a map and perform the SLAM algorithm.

UPDATE:

I had some hard time trying to get the LiDAR to work because I get the error 80008002! which refers to low voltage.
I had to start the node multiple times in order to get the LiDAR wotking again.

I moved on to the autonomous navigaiton part and had to change some settings in order to get it to work. The launch file "nav_3" had all the necessary parameters to start but there was some missing tf trees that I had to add according to the warning that I got when I launch it. I had to add two or three missing tf links.
The autonomous navigation seems to work but there is a problem that I think I have a solution to. The problem is that when I choose a place for the robot to go to, it can control the DC motor but not the servomotor. When I "echoed" the topic "/cmd/vel" that the algorithm uses to move the car, I saw that the part responsible for the servomotor "angular.z" never reaches 1 or -1 (it was -0.6 last time I checked). However, in the arduino code that receives the "/cmd/vel" command, it is set to turn the servomotor only when it receives the aforementioned values. So, I will have to modify that code in order to have an incremental increase or decrease in values to turn the servomotor at the desired direction.
