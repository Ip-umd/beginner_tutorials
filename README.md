## Programming Assignment: ROS Publisher/Subscriber 
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
Implementation of ROS Publisher and Subscriber in (C++).
- talker.cpp(Publisher)
- listener.cpp(Subscriber)

## Dependencies

- Ubuntu 16.04
- ROS Kinetic

## Build Instructions
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Ip-umd/beginner_tutorials.git
cd ..
catkin_make
```

## Running the code

Open a new terminal and run following command:
```
roscore
```

Instructions for running publisher:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials talker
```

Instructions for running subscriber:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```

## Running ROS launch 
To run the launch file, run following commands (freq parameter is the loop rate of the publisher node):

```
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch freq:=<frequency>
```

## Running Service

```
cd <path to catkin_ws>
source devel/setup.bash
rosservice call /change_string "ENPM 808X"

```

