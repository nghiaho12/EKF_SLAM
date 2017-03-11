This is an implementation of the EKF SLAM with known data association algorithm from the book Probabilistic Robotics by Thrun et. al. I wrote this code to help me understand the material better.

The simulation consists of a robot with a range sensor that can detect landmarks in the world. You move the robot around using the arrow keys.

![EKF_SLAM](https://cloud.githubusercontent.com/assets/1471705/23823871/a291bbde-06bf-11e7-8d3b-7cf49543fc94.png)

###Requirements
This demo code requires the following libraries
- SDL2
- Eigen

I've only tested this on a Linux machine but it should on Mac and Windows because both libraries are cross platform.

###Compiling
```
mkdir build
cd build
cmake ..
make
```
###Customization
All the parameters for the simulation can be found in config.h
