/**
 * @file listener_node.cpp
 * @author Sakshi Kakde
 * @brief Code to subscibe to a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <beginner_tutorials/listener.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle* nh_p = new ros::NodeHandle("~");
  Listener listener(nh_p);
  listener.runNode();
  delete nh_p;
  return 0;
}
