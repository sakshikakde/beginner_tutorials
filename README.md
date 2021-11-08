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

## How to run the code(without roslaunch)
1) Change the directory 

``` 
cd catkin_ws

```
2)Source the workspace

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
roslaunch beginner_tutorials talker_chatter.launch
```

### Parameters

- platform_name : default="sakshi"
- publisher_topic_name :  default="/chatter"
- publisher_rate : default = 20
- service_name :  default="add_two_ints"
- subscriber_topic_name : default="/chatter"
