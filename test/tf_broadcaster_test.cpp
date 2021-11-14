/**
 * @file test_broadcaster.cpp
 * @author Sakshi Kakde
 * @brief A code to test the tf broadcaster
 * @version 0.1
 * @date 2021-10-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include <gtest/gtest.h>
#include <tf/transform_listener.h>
#include <beginner_tutorials/talker.hpp>

std::shared_ptr<ros::NodeHandle> nh;

TEST(TESTSuite, tf_broadcaster) {
  std::string platform_name, parent_frame_name, child_frame_name;
  nh->param<std::string>("broadcaster_name", platform_name, "talker_test");
  double x_expected = 10.0;
  double y_expected = 10.0;
  double z_expected = 10.0;

  double roll_expected = 0;
  double pitch_expected = 0;
  double yaw_expected = 0;
  ros::param::get(platform_name + "/parent_frame_name", parent_frame_name);
  ros::param::get(platform_name + "/child_frame_name", child_frame_name);

  ros::param::get(platform_name + "/x", x_expected);
  ros::param::get(platform_name + "/y", y_expected);
  ros::param::get(platform_name + "/z", z_expected);

  ros::param::get(platform_name + "/roll", roll_expected);
  ros::param::get(platform_name + "/pitch", pitch_expected);
  ros::param::get(platform_name + "/yaw", yaw_expected);

  tf::TransformListener listener;
  tf::StampedTransform transform;
  bool tf_exists;
  ros::Duration(1).sleep();
  try {
      listener.lookupTransform(parent_frame_name, child_frame_name,
                                ros::Time(0), transform);
      tf_exists = true;
    }
  catch (tf::TransformException ex) {
    tf_exists = false;
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    }
  EXPECT_TRUE(tf_exists);
  if (tf_exists) {
    double x_listened = transform.getOrigin().getX();
    double y_listened = transform.getOrigin().getY();
    double z_listened = transform.getOrigin().getZ();

    double qx_listened = transform.getRotation().getX();
    double qy_listened = transform.getRotation().getY();
    double qz_listened = transform.getRotation().getZ();
    double qw_listened = transform.getRotation().getW();

    tf::Quaternion q(
        qx_listened,
        qy_listened,
        qz_listened,
        qw_listened);
    tf::Matrix3x3 m(q);
    double roll_listened, pitch_listened, yaw_listened;
    m.getRPY(roll_listened, pitch_listened, yaw_listened);

    EXPECT_NEAR(x_listened, x_expected, 0.01);
    EXPECT_NEAR(y_listened, y_expected, 0.01);
    EXPECT_NEAR(z_listened, z_expected, 0.01);
    EXPECT_NEAR(roll_listened, roll_expected, 0.01);
    EXPECT_NEAR(pitch_listened, pitch_expected, 0.01);
    EXPECT_NEAR(yaw_listened, yaw_expected, 0.01);
  }
}

int main(int argc,
         char **argv) {
  ros::init(argc, argv, "tf_broadcaster_test_node");
  nh.reset(new ros::NodeHandle);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

