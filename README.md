# Autocar
An 1/10 autocar project on Advance Robotics class with our team Robotgrandson.

Instructor: Prof. Christoffer Heckman.
Team Member: Chi-Ju Wu, Chi Chen, Zheng Shen, Dongming Chang, Zhengyu Hua.


Goal:

Realize a self-driving race car which is capable of many different challenges and integrate the hardware and algorithms in perception, motion planning and trajectory generation. 

Hardware and design:

The hardware includes a RealSense camera, a Pololu controller, an on board IMU and a Rock 64 CPU. This system utilizes the depth data of RealSense camera to realize real-time obstacle detection and IMU data to track the status of the race car. We designs a two-stage shelf for all the devices, as the picture showed.
![](picture/car1.jpg)
Layout of wiring of the race car
![](picture/car2.jpg)
The outlook of the race car

Chanllenges:

Results:

The implementation of our designed algorithm demonstrates that the race car realizes moving at a fast speed on straight road, the capability of detecting walls or obstacles and reacting the properly before collision. Even collision happens, the implementation also shows the ability of back out from the collision and continue driving normally. The future work includes realization of real-time visual SLAM and improving performance of driving in a relative complex environment. 

How to import our code:
The IMU package is a subscriber for ROS to get some location info of our auto car, for example the turning angle.
The Image package has the code for RealSense camera depth information. We took some blocks on the image to represent the distance from left, right and straight. This would be used at determine whether the car needs to make a turn and adjust its angle when not going very straight.

Credit to our team.
