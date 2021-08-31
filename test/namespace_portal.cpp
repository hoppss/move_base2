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
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10, std::bind(&MinimalSubscriber::costmap_callback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&MinimalSubscriber::laser_callback, this, _1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/ns1/scan", 10);

    vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/ns1/cmd_vel", 10, std::bind(&MinimalSubscriber::vel_callback, this, _1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    init_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ns1/initialpose", 10);

    timer_ = this->create_wall_timer(1000ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "costmap %ld", msg->header.stamp.nanosec);
  }

  // scan transfer
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "scan %ld", msg->header.stamp.nanosec);
    sensor_msgs::msg::LaserScan temp;
    temp = *msg;

    scan_pub_->publish(temp);
  }

  // scan transfer
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "vel %f", msg->linear.x);

    geometry_msgs::msg::Twist temp;
    temp = *msg;

    vel_pub_->publish(temp);
  }

  // initial pose
  rclcpp::TimerBase::SharedPtr timer_;
  void timer_callback()
  {
    static int cnt = 0;

    ++cnt;

    if (cnt == 3) {
      RCLCPP_INFO(get_logger(), "publish initpose");
      geometry_msgs::msg::PoseWithCovarianceStamped p;
      p.header.stamp = now();
      p.header.frame_id = "map";

      p.pose.pose.position.x = -2;
      p.pose.pose.position.y = -0.5;
      init_pose_pub_->publish(p);
      timer_->cancel();
    }
  }
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
