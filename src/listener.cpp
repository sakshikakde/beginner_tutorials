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
     this->service_name, "/add_two_ints");
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
