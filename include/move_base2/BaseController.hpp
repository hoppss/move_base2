// Copyright (c) 2021 Xiaomi Corporation
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

#ifndef MOVE_BASE2__BASECONTROLLER_HPP_
#define MOVE_BASE2__BASECONTROLLER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "motion_msgs/msg/frameid.hpp"
#include "motion_msgs/msg/se3_velocity_cmd.hpp"

namespace move_base
{
class BaseController
{
public:
  BaseController();
  ~BaseController();

  void
  initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent,
             const std::shared_ptr<tf2_ros::Buffer>& tf,
             const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
                 vel_publisher,
             const rclcpp_lifecycle::LifecyclePublisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr
                 body_cmd_publisher);

  bool rotate(double);

  // tools
  bool getCurrentPose(geometry_msgs::msg::PoseStamped& odom_pose);
  bool transformPose(const std::string& target_frame,
                     const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose);

  bool approachOnlyRotate(const geometry_msgs::msg::PoseStamped& target);

private:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr body_cmd_pub_;
};

}  // namespace move_base

#endif  // MOVE_BASE2__BASECONTROLLER_HPP_
