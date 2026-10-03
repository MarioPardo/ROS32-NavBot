
Using STM32 + ROS2 to create a robot that maps + navigates a room

# Goal
Bit by bit, create a robot that is able to navigate using a combination of onboard controls and ROS based logic. We will implement simple driving, to then SLAM, and autonomous navigation.
I plan to add more intelligent navigation, such as using camera(s) to find important landmarks and gain a semantic understanding of it's environment. 

## Hardware: 
* Raspberry Pi (runs ROS2)
* STM32 BluePill (onboard embedded control)

## Based off of Course
This is based on Antonio Brandi's series of ROS2 [Udemy Courses]https://www.udemy.com/user/antonio-brandi/)
My "spin" on things: the course's robot runs on Arduino. I translate this to run on STM32 running FreeRTOS. 
Also, the eventual landmark recognition and semantic understanding


# General Flow

ROS2 on PC <--Wifi--> Raspi <--UART--> STM32

##ROS2
[*] Takes our joystick inputs, and maps this to requested wheel velocities from the robot
[*] Runs a simulated version of the robot
[] Mapping, Navigation, etc

## Raspi
* Runs on the same network as PC, so takes ROS topics and translates them into custom UART data for the STM32
[] Same but in reverse, makes STM32 odometry (etc) data available for ROS2 on PC

## STM32
Handles the low level control of the robot: controlling the motors, and reporting back with odometry data
* includes custom battery management circuits, etc


 
