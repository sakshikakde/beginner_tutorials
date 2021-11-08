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
    initServices();
}

Talker::~Talker() {
      delete this->nh_p;
}

void Talker::initParams() {
    this->nh_p->param<std::string>("publisher_topic_name",
     this->publisher_topic_name, "/chatter");
    this->nh_p->param<int>("publisher_rate",
     this->publisher_rate, 10);
    this->nh_p->param<std::string>("service_name",
     this->service_name, "add_two_ints");
}

void Talker::initPublishers() {
    this->chatter_pub = this->nh_p->advertise<std_msgs::String>(
        this->publisher_topic_name,
         this->publisher_rate, this);
}

void Talker::initServices() {
  this->service = this->nh_p->advertiseService(this->service_name, &Talker::add, this);
  ROS_INFO_STREAM("Ready to add two ints.");
}

bool Talker::add(beginner_tutorials::AddTwoInts::Request  &req,
        beginner_tutorials::AddTwoInts::Response &res) {
  if ((req.a < 0) || (req.b < 0)) {
    ROS_WARN_STREAM("One of the numbers is negative");
  }
  res.sum = req.a + req.b;
  if (res.sum == 0) {
    ROS_ERROR_STREAM("Sum is 0!");
  }
  ROS_INFO_STREAM("request: x = " << req.a << " y = " <<  req.b);
  ROS_INFO_STREAM("sending back response: " << res.sum);
  return true;
}

void Talker::runNode() {
  int count = 0;
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Talker node is active");
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
