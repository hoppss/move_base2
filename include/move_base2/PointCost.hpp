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

#ifndef MOVE_BASE2__POINTCOST_HPP_
#define MOVE_BASE2__POINTCOST_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace move_base
{
class PointCost
{
public:
  PointCost();
  ~PointCost() = default;

  void initialize(
    const nav2_util::LifecycleNode::SharedPtr & nh,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  bool isValidPose(const geometry_msgs::msg::PoseStamped & p, bool allow_unknown = true);
  unsigned char getPointCost(const geometry_msgs::msg::PoseStamped & p);

  // false is collision
  bool collisionFreeCheck(const nav_msgs::msg::Path & path, double & sum_dist);

private:
  nav2_util::LifecycleNode::SharedPtr nh_;
  rclcpp::Logger logger_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;

  double check_distance_;
};

}  // namespace move_base

#endif  // MOVE_BASE2__POINTCOST_HPP_
