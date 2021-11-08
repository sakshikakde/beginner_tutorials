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
}

Listener::Listener(ros::NodeHandle* nh_p) {
    this->nh_p = nh_p;
    initParams();
    initSubscribers();
    initServiceClient();
}

Listener::~Listener() {
    delete this->nh_p;
}

void Listener::initParams() {
    this->nh_p->param<std::string>("subscriber_topic_name",
     this->subscriber_topic_name, "/chatter");
    this->nh_p->param<std::string>("service_name",
     this->service_name, "/talker_node/add_two_ints");
}

void Listener::initSubscribers() {
    this->chatter_sub = this->nh_p->subscribe(this->subscriber_topic_name,
     1, &Listener::chatter_callback, this);
}

void Listener::initServiceClient() {
    this->client = this->nh_p->serviceClient<beginner_tutorials::AddTwoInts>(
        this->service_name,
        this);
}
void Listener::chatter_callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO_STREAM("Yes, I heard " << msg->data.c_str());
    ROS_INFO_STREAM("Can you add two numbers for me?");
    beginner_tutorials::AddTwoInts srv;
    srv.request.a = 10;
    srv.request.b = -10;
    if (client.call(srv)) {
        ROS_INFO_STREAM("Yes! The sum is : " << srv.response.sum);
    } else {
        ROS_ERROR_STREAM("Failed to call service add_two_ints");
    }
}

void Listener::runNode() {
  ROS_DEBUG_ONCE("Talker node activated.");
  ros::spin();
}
