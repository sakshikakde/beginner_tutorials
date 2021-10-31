/**
 * @file talker.hpp
 * @author Sakshi Kakde
 * @brief A class to publish string data on a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef INCLUDE_BEGINNER_TUTORIALS_TALKER_HPP_
#define INCLUDE_BEGINNER_TUTORIALS_TALKER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

class Talker {
 public:
    /**
     * @brief Construct a new Talker object
     * 
     */
    Talker();
    /**
     * @brief Destroy the Talker object
     * 
     */
    ~Talker();
    /**
     * @brief Runs the ros::ok loop and publishes data at a predefined rate
     * 
     */
    void runNode();
    ros::NodeHandle* nh_p;  // nodehandle
 private:
    std::string publisher_topic_name;  // ROS publisher topic name
    ros::Publisher chatter_pub;  // ROS publisher object
    /**
     * @brief Function to init params
     * 
     */
    void initParams();
    /**
     * @brief Function to init publishers
     * 
     */
    void initPublishers();
};
#endif  // INCLUDE_BEGINNER_TUTORIALS_TALKER_HPP_