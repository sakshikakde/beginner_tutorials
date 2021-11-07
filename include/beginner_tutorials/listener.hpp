/**
 * @file listener.hpp
 * @author Sakshi Kakde
 * @brief A class to subscribe to a topic of type string
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef INCLUDE_BEGINNER_TUTORIALS_LISTENER_HPP_
#define INCLUDE_BEGINNER_TUTORIALS_LISTENER_HPP_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include "beginner_tutorials/AddTwoInts.h"

class Listener {
 public:
    /**
     * @brief default constructor for Listener object
     * 
     */
    Listener();
    /**
     * @brief explicit constructor for Listener object
     * 
     * @param nh_p pointer to nodehandle
     */
    explicit Listener(ros::NodeHandle* nh_p);
    /**
     * @brief Destroy the Listener object
     * 
     */
    ~Listener();
    /**
     * @brief waits for publisher to publish 
     * 
     */
    void runNode();
    ros::NodeHandle* nh_p;  // nodehandle

 private:
    std::string subscriber_topic_name, service_name;  // ROS subscriber topic name
    ros::Subscriber chatter_sub;  // ROS Subscriber object
    ros::ServiceClient client;
    /**
     * @brief Function to init params
     * 
     */
    void initParams();
    /**
     * @brief Function to init subscribers
     * 
     */
    void initSubscribers();
    void initServiceClient();
    /**
     * @brief Callback function for subscriber
     * 
     * @param msg Message received from publisher
     */
    void chatter_callback(const std_msgs::String::ConstPtr& msg);
};
#endif  // INCLUDE_BEGINNER_TUTORIALS_LISTENER_HPP_
