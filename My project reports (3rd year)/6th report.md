In this session I have reached this level in my project:

I still have the probem where if a lot of instruction are sent, it takes time to do them and will do them entirely even with a big delay. This is very probably dur to the code size that is imported to the Arduino R3 board. I remember when uploading the code that I had a warning telling me that the code uses almost 90% of the card's capacity which can lead to some problems.

As of today, the 07/11/2024, I will test the car with a battery (5200mAH 11.1V 80C) that I have received from a friend.

UPDATE:

Great news! Using the new battery, the car is now able to accelerate and move freely even at some very high speeds with the full load.

I had to craft a new connector for the battery in order to plug it in to the car.

I tried to do SLAM with the car fully wirelessly and it seemed to work fine until when the car turns abruptly. The generated map by the LiDAR shifts, creating a map overlapping. Having this issue before during my internship, I knew that this is due to the car turining sideways in a harsh manner. However, a friend of mine suggested another that this issue can be tackeled by using **"SLAM particle sampling"**. So this will be the main subject I will be working on next time.
