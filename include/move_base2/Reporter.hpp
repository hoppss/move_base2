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

#ifndef MOVE_BASE2__REPORTER_HPP_
#define MOVE_BASE2__REPORTER_HPP_
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "automation_msgs/msg/nav_status.hpp"

namespace move_base
{
class Reporter
{
public:
  Reporter();
  ~Reporter();

  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent);
  void report(const int mode, const uint8_t status, std::string description);

private:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<automation_msgs::msg::NavStatus>
  ::SharedPtr reporter_pub_;
};

}  // namespace move_base

#endif  // MOVE_BASE2__REPORTER_HPP_
