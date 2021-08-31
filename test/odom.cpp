// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <iostream>
#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom_out", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_out", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time t = rclcpp::Time(msg->header.stamp);
    double dt = t.seconds() - last_stamp_.seconds();
    if (dt > 0.04) {
      RCLCPP_ERROR(
        this->get_logger(), "dt %f, speed[%f, %f]", dt, msg->twist.twist.linear.x,
        msg->twist.twist.angular.z);
    } else {
    }

    last_stamp_ = t;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Time last_stamp_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
