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
#include <memory>
#include "move_base2/PointCost.hpp"

namespace move_base
{
PointCost::PointCost()
: logger_(rclcpp::get_logger("move_base_pointcost"))
{
  check_distance_ = 2.5;
}

void PointCost::initialize(
  const nav2_util::LifecycleNode::SharedPtr & nh,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  nh_ = nh;

  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
}

bool PointCost::isValidPose(const geometry_msgs::msg::PoseStamped & p, bool allow_unknown)
{
  auto cost = getPointCost(p);
  bool valid = true;
  if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
    cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
    (!allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION))
  {
    valid = false;
  }
  return valid;
}

unsigned char PointCost::getPointCost(const geometry_msgs::msg::PoseStamped & p)
{
  unsigned int cell_x, cell_y;
  if (!costmap_->worldToMap(p.pose.position.x, p.pose.position.y, cell_x, cell_y)) {
    RCLCPP_WARN(logger_, "PoseCost, costmap.worldToMap failed");
    return nav2_costmap_2d::LETHAL_OBSTACLE;
  }
  unsigned char cost = costmap_->getCost(cell_x, cell_y);

  return cost;
}

bool PointCost::collisionFreeCheck(const nav_msgs::msg::Path & path, double & sum_dist)
{
  // 周期控制, 1s 检测一次，
  // static uint seq = 0;
  // if (seq++ % 2 != 0)
  //   return true;

  // 1. get robot pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose)) {
    RCLCPP_WARN(logger_, "PointCost, costmap get robotpose failed");
    return false;
  }

  // 2. get closest index in ref path
  int path_size = path.poses.size();

  double closest_dist = 1e9;
  int closest_index = 1;

  for (int i = 1; i < path_size; i += 2) {
    double d = nav2_util::geometry_utils::euclidean_distance(current_pose, path.poses[i]);
    if (d < closest_dist) {
      closest_dist = d;
      closest_index = i;
    }
  }

  int pre_index = closest_index;

  // 3. iterator to forward check pointcost
  for (int i = closest_index + 1; i < path_size && sum_dist < check_distance_; i += 1) {
    sum_dist += nav2_util::geometry_utils::euclidean_distance(path.poses[pre_index], path.poses[i]);
    pre_index = i;
    unsigned char cost = getPointCost(path.poses[i]);

    /*
    RCLCPP_INFO(nh_->get_logger(), "currpos [%f,%f], pos [%f,%f], cost %d, sum_dist %f", current_pose.pose.position.x,
                current_pose.pose.position.y, path.poses[i].pose.position.x, path.poses[i].pose.position.y, (int)cost,
                sum_dist);
    */
    if (sum_dist < 1.0 &&
      (cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
      cost == nav2_costmap_2d::LETHAL_OBSTACLE))
    {
      RCLCPP_WARN(
        logger_, "PointCost-Neighbor, cost %d; dist %f, pose [%f, %f], let's replan", cost,
        sum_dist,
        path.poses[i].pose.position.x, path.poses[i].pose.position.y);
      return false;
    } else if (cost >= 220 && cost <= nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(
        logger_, "PointCost-Remote, cost %d; dist %f, pose [%f, %f], let's replan", cost, sum_dist,
        path.poses[i].pose.position.x, path.poses[i].pose.position.y);
      return false;
    }
  }

  return true;
}

}  // namespace move_base
