/**
 * @file talker.cpp
 * @author Sakshi Kakde
 * @brief A class to publish string data on a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <beginner_tutorials/talker.hpp>

Talker::Talker() {
}

Talker::Talker(ros::NodeHandle* nh_p) {
    this->nh_p = nh_p;
    initParams();
    initPublishers();
}

Talker::~Talker() {
      delete this->nh_p;
}

void Talker::initParams() {
    this->nh_p->param<std::string>("publisher_topic_name",
     this->publisher_topic_name, "/chatter");
    this->nh_p->param<int>("publisher_rate",
     this->publisher_rate, 10);
}

void Talker::initPublishers() {
    this->chatter_pub = this->nh_p->advertise<std_msgs::String>(
        this->publisher_topic_name,
         this->publisher_rate, this);
}

void Talker::runNode() {
  int count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Can you hear " << count << " ?";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    this->chatter_pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
