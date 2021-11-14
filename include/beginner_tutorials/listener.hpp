/**
 * MIT License
 *
 * Copyright (c) 2021 Sakshi Kakde
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
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
    std::string subscriber_topic_name,
     service_name;  // ROS subscriber topic name
    ros::Subscriber chatter_sub;  // ROS Subscriber object
    ros::ServiceClient client;  // ROS service client object
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
    /**
     * @brief Function to init service clients
     * 
     */
    void initServiceClient();
    /**
     * @brief Callback function for subscriber
     * 
     * @param msg Message received from publisher
     */
    void chatter_callback(const std_msgs::String::ConstPtr& msg);
};
#endif  // INCLUDE_BEGINNER_TUTORIALS_LISTENER_HPP_
