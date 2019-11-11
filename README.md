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

## Inspecting TF frames
Considering that the above launch file is still running, run following commands to verify the TF frames using tf_echo. 
Running the tf_echo command will give the tranform of the /talk frame with respect to the /world frame.

```
rosrun tf tf_echo /world /talk
```

It gives following output:
```
At time 1573496740.701
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.000, 0.000, 0.938, 0.347]
            in RPY (radian) [0.000, -0.000, 2.434]
            in RPY (degree) [0.000, -0.000, 139.437]
At time 1573496741.402
- Translation: [1.000, 2.000, 3.000]
- Rotation: in Quaternion [0.000, 0.000, 0.938, 0.347]
            in RPY (radian) [0.000, -0.000, 2.434]
            in RPY (degree) [0.000, -0.000, 139.437]
```

To verify the TF frames using rqt_tf_tree, run the following command in a new terminal:
```
rosrun rqt_tf_tree rqt_tf_tree
```

To get the PDF output of the view_frames tool, run the following command in new terminal:
```
rosrun tf view_frames
evince frames.pdf
```
## Running rostest
First build using the following commands:
```
sudo killall -9 roscore   
sudo killall -9 rosmaster 
cd <path to catkin_ws>
source devel/setup.bash
catkin_make run_tests_beginner_tutorials
```

Now run the test using the test launch file:
```
rostest beginner_tutorials changeStringTest.launch frequency:=10
```

It will give following output:
```
... logging to /home/ishan/.ros/log/rostest-ishan-OMEN-by-HP-Laptop-19344.log
[ROSUNIT] Outputting test results to /home/ishan/.ros/test_results/beginner_tutorials/rostest-test_changeStringTest.xml
[ INFO] [1573492356.593715521]: Frequency : 10
[ INFO] [1573492356.593782422]: Base string changed.0
[ INFO] [1573492356.693862236]: Base string changed.1
[ INFO] [1573492356.793855920]: ENPM808X2
[ INFO] [1573492356.893853075]: ENPM808X3
[ INFO] [1573492356.993874406]: ENPM808X4
[ INFO] [1573492357.093915247]: ENPM808X5
[Testcase: testchangeStringTest] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-changeStringTest/checkChangeString][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

rostest log file is in /home/ishan/.ros/log/rostest-ishan-OMEN-by-HP-Laptop-19344.log

```
## Bag files
###  Recording bag files with the launch file

Run the following commands to record bag files. The bag files are stored in results/rosbagRecord.bag .
```
sudo killall -9 roscore  
sudo killall -9 rosmaster 
cd <path to catkin_ws>
source devel/setup.bash
roslaunch beginner_tutorials beginner_tutorials.launch freq:=<frequency> enableRecord:=true
```
To disable bag file recording, pass false to the argument enableRecord i.e.  "enableRecord:=false"

### Inspecting the bag file
Run the following command to inspect the bag file:
```
cd <path to repository>/results
rosbag info rosbagRecord.bag
```
It will give output as follows:
```
path:        rosbagRecord.bag
version:     2.0
duration:    23.3s
start:       Nov 11 2019 12:50:57.00 (1573494658.00)
end:         Nov 11 2019 12:51:21.34 (1573494681.34)
size:        160.3 KB
messages:    713
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      116 msgs    : std_msgs/String   
             /rosout       242 msgs    : rosgraph_msgs/Log  (4 connections)
             /rosout_agg   239 msgs    : rosgraph_msgs/Log 
             /tf           116 msgs    : tf2_msgs/TFMessage

```
### Playing back the bag file with the Listener node demonstration
First, launch the listener node using following commands given that roscore is running:
```
cd ~/catkin_ws
source ./devel/setup.bash
rosrun beginner_tutorials listener
```
Now, play back the bag file using following command:
```
cd <path to repository>/results
rosbag play rosbagRecord.bag
```

