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
    this->nh_p->param<std::string>("parent_frame_name",
     this->parent_frame_name, "/world");
    this->nh_p->param<std::string>("child_frame_name",
     this->child_frame_name, "/talk");
    this->nh_p->param<double>("x",
     this->x, 10.0);
    this->nh_p->param<double>("y",
     this->y, 20.0);
    this->nh_p->param<double>("z",
     this->z, 30.0);
    this->nh_p->param<double>("roll",
     this->roll, 50.0);
    this->nh_p->param<double>("pitch",
     this->pitch, 50.0);
    this->nh_p->param<double>("yaw",
     this->yaw, 50.0);
}

void Talker::initPublishers() {
    this->chatter_pub = this->nh_p->advertise<std_msgs::String>(
        this->publisher_topic_name,
         this->publisher_rate, this);
}

void Talker::initServices() {
  this->service = this->nh_p->advertiseService(this->service_name,
   &Talker::add,
   this);
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

void Talker::broadcastTransform() {
  ROS_INFO_STREAM("Broadcasting transform");
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(this->x, this->y, this->z));
  tf::Quaternion q;
  q.setRPY(this->roll, this->pitch, this->yaw);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform,
  ros::Time::now(),
  this->parent_frame_name,
  this->child_frame_name));
}

void Talker::runNode() {
  int count = 0;
  ros::Rate loop_rate(this->publisher_rate);
  while (ros::ok()) {
    ROS_DEBUG_STREAM("Talker node is active");
    this->broadcastTransform();
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Can you hear " << count << " ?";
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());
    this->chatter_pub.publish(msg);
    uint32_t n_sub = this->chatter_pub.getNumSubscribers();
    if (n_sub == 0) {
      ROS_FATAL_STREAM("NO SUBSCRIBERS!");
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
}
