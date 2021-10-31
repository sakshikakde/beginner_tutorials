/**
 * @file listener.cpp
 * @author Sakshi Kakde
 * @brief A class to subscribe to a topic of type string
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <beginner_tutorials/listener.hpp>

Listener::Listener() {
    this->nh_p = new ros::NodeHandle("~");
    initParams();
    initSubscribers();
}

Listener::~Listener()
{}

void Listener::initParams() {
    this->nh_p->param<std::string>("subscriber_topic_name", this->subscriber_topic_name, "/chatter");
}

void Listener::initSubscribers() {
    this->chatter_sub = this->nh_p->subscribe(this->subscriber_topic_name, 1, &Listener::chatter_callback, this);
}

void Listener::chatter_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Yes, I heard [%s]", msg->data.c_str());
}

void Listener::runNode() {
  ros::spin();
}
