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
  Talker talker;
  talker.runNode();
  return 0;
}
