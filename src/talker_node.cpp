/**
 * @file talker_node.cpp
 * @author Sakshi Kakde
 * @brief Code to publish string data on a topic
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <beginner_tutorials/talker.hpp>

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker_node");
  ros::NodeHandle* nh_p = new ros::NodeHandle("~");
  Talker talker(nh_p);
  talker.runNode();
  delete nh_p;
  return 0;
}
