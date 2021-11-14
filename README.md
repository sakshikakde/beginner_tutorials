[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
---
# beginner_tutorials
## Introduction
This repository provides a simple example to run ROS publisher and sibscriber

## File Structure
- include       
-- listener.hpp    
-- talker.hpp   

- src    
-- listener.cpp    
-- talker.cpp    
-- listener_node.cpp     
-- talker_node.cpp     

listener_node.cpp and talker_node.cpp are the files where the objectes for the classes Listener and Talker are created.

## Building the package
1) Create a catkin workspace catkin_ws
2) Clone the package inside catkin_ws/src using 

``` 
git clone https://github.com/sakshikakde/beginner_tutorials.git 
```
4) Change the directory
```
cd catkin_ws

```
5) run 

``` 
catkin build 
```

## How to run the code
1) Change the directory 

``` 
cd catkin_ws

```
2) Source the workspace

```
sourcedevel/setup.bash
```
3) In a terminal, run 
```
roscore
```
5) In a new terminal, run the talker by using

```
rosrun beginner_tutorials talker

```
5) In a new terminal, run the listener by using
```
rosrun beginner_tutorials listener
```
## How to run the code(with launch)

``` 
cd catkin_ws

```
2)Source the workspace

```
sourcedevel/setup.bash
```
3) In a terminal, run 
```
roslaunch beginner_tutorials talker_chatter.launch platform_name:="my_laptop"
```

### Parameters

- platform_name : default="sakshi"
- publisher_topic_name :  default="/chatter"
- publisher_rate : default = 20
- service_name :  default="add_two_ints"
- subscriber_topic_name : default="/chatter"


## ROS Service from command line
Use the following command to call the ros service from terminal:

```
rosservice call /add_two_ints "a: 10
b: 30" 
```

## Run rostest

1) Change the directory to the workspace
``` 
cd catkin_ws

```
3) Build the workspace using the following command:     
```
catkin build --catkin-make-args test      
```     
3)Source the workspace

```
source devel/setup.bash
```
4) In a terminal, run 
```
rostest beginner_tutorials test_broadcaster.launch
```
## Recording the bag file
1) Change the directory to the workspace
``` 
cd catkin_ws
```    
2) Source the workspace

```
source devel/setup.bash
```
3) In a terminal, run 
```
roslaunch beginner_tutorials talker_chatter.launch record_bag:=true bags_directory:=/home/sakshi/courses/ENPM808X/week11/catkin_ws/src/beginner_tutorials/results/bag

```
Note that you need to update the bags_directory param to a suitable location

## Running the bag file
1) Change the directory to the location where rosbag is recorded
 
2) Run the command

```
rosbag play <bagfile_name> --pause
```
3) In a new terminal, change the directory to the workspace
``` 
cd catkin_ws
```    
4) Source the workspace

```
source devel/setup.bash
```
5) Run 
```
rosrun beginner_tutorials listener

```

6) Unpause the bag file. the listener node should be able to get the topic messages from the bag file.
