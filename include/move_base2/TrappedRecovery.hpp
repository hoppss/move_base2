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

#ifndef MOVE_BASE2__TRAPPEDRECOVERY_HPP_
#define MOVE_BASE2__TRAPPEDRECOVERY_HPP_

#include <string>
#include <memory>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "ception_msgs/msg/around.hpp"

// #include "move_base2/PointCost.hpp"
#include "move_base2/line_iterator.hpp"
#include "angles/angles.h"

#include "eigen3/Eigen/Eigen"

namespace move_base
{
class TrappedRecovery
{
public:
  TrappedRecovery();
  ~TrappedRecovery();

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  void setMode(int i);

  bool isTrapped();
  bool isTrappedInPose(double x, double y, double yaw);
  bool isUltrasonicCurrent();

  //   bool runRecovery();

  // 8-direction， clockwise
  //   enum Direction
  //   {
  //     Front_Left = 0,
  //     Front_Middle = 1,
  //     Front_Right = 2,
  //     Middle_Right = 3,
  //     Back_Right = 4,
  //     Back_Middle = 5,
  //     Back_Left = 6,
  //     Middle_Left = 7,
  //   };

  void transformFootprint(
    double x, double y, double theta,
    const std::vector<geometry_msgs::msg::Point> & footprint_spec,
    std::vector<geometry_msgs::msg::Point> & oriented_footprint);

  double lineCost(int x0, int x1, int y0, int y1);

  // collision check interface
  double scoreFootprint(std::vector<geometry_msgs::msg::Point> oriented_footprint);
  bool transformPose(
    const std::string frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose);

  double getPointCost(int x, int y);

  // ultrasonic, false is collision
  bool collisionFreeCheck(const nav_msgs::msg::Path & path, double & sum_dist);
  bool ultrasonicFrontFree();

private:
  int mode_;
  std::string global_frame_;
  std::string base_frame_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> controller_costmap_;

  std::vector<geometry_msgs::msg::Point> footprint_;

  rclcpp::Logger logger_;
  //   std::map<Direction, double> cost_map_;
  //   std::map<Direction, geometry_msgs::msg::Point> footprint_map_;
  //   std::map<int, Eigen::Vector2d> direction_map_;
  //   void initFootprint();
  //   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
  //        escapte_direction_pub_;
  //   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> escape_path_pub_;
  //   void publishMarker(const std::vector<Eigen::Vector2d>& pts, double angle, int i = 0);

  // rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // ultrasonic subscribe_
  rclcpp::Subscription<ception_msgs::msg::Around>::SharedPtr ultrasonic_sub_;
  void ultrasonicCallback(const ception_msgs::msg::Around::SharedPtr msg);
  sensor_msgs::msg::Range last_ultrasonic_range_;
};

}  // namespace move_base
#endif  // MOVE_BASE2__TRAPPEDRECOVERY_HPP_
