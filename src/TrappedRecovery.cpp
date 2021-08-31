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
#include <vector>
#include <algorithm>
#include <string>

#include "move_base2/TrappedRecovery.hpp"

namespace move_base
{
TrappedRecovery::TrappedRecovery()
: controller_costmap_(nullptr), logger_(rclcpp::get_logger("move_base_trapped"))
{
  // 0 is AB-NAV
  // 1 is TRACKING
  mode_ = 0;
}

TrappedRecovery::~TrappedRecovery()
{
}

void TrappedRecovery::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & nh,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = nh;
  //   auto node = node_.lock();
  //   if (!node)
  //   {
  //     throw std::runtime_error{ "Failed to lock node" };
  //   }

  clock_ = node_->get_clock();

  tf_ = tf;
  controller_costmap_ = costmap_ros;
  global_frame_ = controller_costmap_->getGlobalFrameID();
  base_frame_ = controller_costmap_->getBaseFrameID();

  // vel_pub_ = vel_pub;
  RCLCPP_INFO(
    logger_, "global_frame %s, local_frame %s", global_frame_.c_str(),
    base_frame_.c_str());

  footprint_ = controller_costmap_->getRobotFootprint();

  for (size_t i = 0; i < footprint_.size(); ++i) {
    RCLCPP_INFO(
      logger_, "i %d, [%f, %f, %f]", i, footprint_[i].x, footprint_[i].y,
      footprint_[i].z);
  }
  // initFootprint();

  //   escapte_direction_pub_ =
  //      node_->create_publisher<visualization_msgs::msg::Marker>("escape_direction", 1);
  //   escapte_direction_pub_->on_activate();

  //   escape_path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("escape_path", 1);
  //   escape_path_pub_->on_activate();

  // ultrasonic
  ultrasonic_sub_ = node_->create_subscription<ception_msgs::msg::Around>(
    "ObstacleDetection", rclcpp::SystemDefaultsQoS(),
    std::bind(&TrappedRecovery::ultrasonicCallback, this, std::placeholders::_1));
}

void TrappedRecovery::setMode(int i)
{
  mode_ = i;
}

bool TrappedRecovery::isTrapped()
{
  // 1. get current pose in odom frame, in odom
  geometry_msgs::msg::PoseStamped global_pose;
  if (!controller_costmap_->getRobotPose(global_pose)) {
    RCLCPP_ERROR(logger_, "runRecovery, getRobotPose failed");
    return false;  // inner error, ignore
  }
  // 2. get oriented_footprint
  std::vector<geometry_msgs::msg::Point> oriented_footprint;

  double yaw = tf2::getYaw(global_pose.pose.orientation);
  transformFootprint(
    global_pose.pose.position.x, global_pose.pose.position.y, yaw, footprint_,
    oriented_footprint);

  if (oriented_footprint.empty()) {
    RCLCPP_ERROR(logger_, "runRecovery, transform footprint failed - empty");
    return false;  // inner error, ignore
  } else {
    RCLCPP_INFO(logger_, "oriented footprint size %d", oriented_footprint.size());
  }

  if (scoreFootprint(oriented_footprint) == -1) {
    RCLCPP_WARN(logger_, "Footprint trapped in obstacle map");
    return true;
  }

  return false;
}

bool TrappedRecovery::isTrappedInPose(double x, double y, double yaw)
{
  std::vector<geometry_msgs::msg::Point> oriented_footprint;

  transformFootprint(x, y, yaw, footprint_, oriented_footprint);

  if (oriented_footprint.empty()) {
    RCLCPP_ERROR(logger_, "runRecovery, transform footprint failed - empty");
    return false;  // inner error, ignore
  } else {
    RCLCPP_INFO(logger_, "oriented footprint size %d", oriented_footprint.size());
  }

  if (scoreFootprint(oriented_footprint) == -1) {
    RCLCPP_WARN(logger_, "Footprint trapped in obstacle map");
    return true;
  }

  return false;
}

// bool TrappedRecovery::runRecovery()
// {
//   // 1. get current pose in odom frame, in odom
//   geometry_msgs::msg::PoseStamped global_pose;
//   if (!controller_costmap_->getRobotPose(global_pose))
//   {
//     RCLCPP_ERROR(logger_, "runRecovery, getRobotPose failed");
//     return false;
//   }

//   // 2. get oriented_footprint
//   std::vector<geometry_msgs::msg::Point> oriented_footprint;

//   double yaw = tf2::getYaw(global_pose.pose.orientation);
//   transformFootprint(global_pose.pose.position.x, global_pose.pose.position.y, yaw,
//                      footprint_, oriented_footprint);

//   if (oriented_footprint.empty())
//   {
//     RCLCPP_ERROR(logger_, "runRecovery, transform footprint failed");
//     return false;
//   }
//   else
//   {
//     RCLCPP_INFO(logger_, "oriented footprint size %d", oriented_footprint.size());
//   }

//   // 3. save all ilegal cell on footprint cells and collison check
//   std::vector<Eigen::Vector2i> ilegal_cells_coords;
//   std::vector<double> ilegal_cells_costs;

//   double cost = scoreFootprint(oriented_footprint, ilegal_cells_coords, ilegal_cells_costs);

//   if (cost < 0)
//   {
//     RCLCPP_FATAL(logger_, "runRecovery, inner error");
//     return false;
//   }
//   else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
//            cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
//   {
//     RCLCPP_ERROR(logger_, "runRecovery, robot is trapped");
//     std::vector<geometry_msgs::msg::Point> ilegal_points;
//     for (size_t i = 0; i < ilegal_cells_coords.size(); ++i)
//     {
//       geometry_msgs::msg::Point p;
//       controller_costmap_->getCostmap()->mapToWorld(ilegal_cells_coords[i](0),
//                                                     ilegal_cells_coords[i](1), p.x, p.y);
//       ilegal_points.push_back(p);
//       RCLCPP_INFO(logger_, "ilegal cell, index [%d, %d], odom [%f, %f], cost %f",
//       ilegal_cells_coords[i](0), ilegal_cells_coords[i](1), p.x, p.y, ilegal_cells_costs[i]);
//     }

//     if (ilegal_points.empty())
//     {
//       RCLCPP_INFO(logger_, "ilegal point empty?");
//       return false;
//     }

//     // 4. transform ilegal points into base_link and normalization
//     std::vector<Eigen::Vector2d> ilegal_points_in_base;
//     Eigen::Vector2d sync_vector;
//     for (size_t i = 0; i < ilegal_points.size(); ++i)
//     {
//       geometry_msgs::msg::PoseStamped p, p_out;
//       p.header.stamp = rclcpp::Time();
//       p.header.frame_id = controller_costmap_->getGlobalFrameID();
//       p.pose.position = ilegal_points[i];
//       p.pose.orientation.w = 1.0;

//       transformPose(controller_costmap_->getBaseFrameID(), p, p_out);
//       Eigen::Vector2d norm_vector(p_out.pose.position.x, p_out.pose.position.y);
//       // norm_vector.normalize();
//       ilegal_points_in_base.push_back(norm_vector);
//       RCLCPP_INFO(logger_, "[%f, %f], %d", norm_vector(0), norm_vector(1), i);
//       sync_vector += norm_vector;
//     }
//     // synthesis direction, show marker for debug
//     sync_vector.normalize();
//     double angle = std::atan2(sync_vector(1), sync_vector(0));

//     RCLCPP_INFO(logger_, "escape direction, [%f, %f]", sync_vector(0), sync_vector(1));

//     publishMarker(ilegal_points_in_base, angle, 0);

//     // scoreFootprint for escape direction

//     // opposite direction
//     double opposite_angle = angles::normalize_angle(angle + M_PI);
//     std::this_thread::sleep_for(std::chrono::milliseconds(2000));
//     publishMarker(ilegal_points_in_base, opposite_angle, 0);

//     // trajectory detect for desired direction movement
//     std::vector<geometry_msgs::msg::PoseStamped> escape_path_in_base;
//     nav_msgs::msg::Path escape_path_in_odom;
//     escape_path_in_odom.header.frame_id = controller_costmap_->getGlobalFrameID();
//     escape_path_in_odom.header.stamp = clock_->now();

//     for (size_t i = 1; i < 4; ++i)
//     {
//       geometry_msgs::msg::PoseStamped p, p_odom;
//       p.header.frame_id = controller_costmap_->getBaseFrameID();
//       p.header.stamp.sec = p.header.stamp.nanosec = 0.0;
//       p.pose.orientation.w = 1.0;
//       p.pose.position.x = 0.05 * i * std::cos(opposite_angle);
//       p.pose.position.y = 0.05 * i * std::sin(opposite_angle);
//       escape_path_in_base.push_back(p);

//       transformPose(controller_costmap_->getGlobalFrameID(), p, p_odom);
//       escape_path_in_odom.poses.push_back(p_odom);
//     }

//     escape_path_pub_->publish(escape_path_in_odom);

//     // collsion check, 只能检查最外层的点？ 挨着的很有可能也是不安全的
//     std::vector<Eigen::Vector2i> dummy_coords;
//     std::vector<double> dummy_costs;
//     for (size_t i = 2; i < escape_path_in_odom.poses.size(); ++i)
//     {
//       std::vector<geometry_msgs::msg::Point> new_footprint;
//       transformFootprint(escape_path_in_odom.poses[i].pose.position.x,
//                          escape_path_in_odom.poses[i].pose.position.y,
//                          tf2::getYaw(escape_path_in_odom.poses[i].pose.orientation),
//                          footprint_, new_footprint);
//       double c = scoreFootprint(new_footprint, dummy_coords, dummy_costs);

//       if (c >= 0 && c != nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
//              c != nav2_costmap_2d::LETHAL_OBSTACLE)
//       {
//         continue;
//       }
//     }
//     auto max_cost_iterator = std::find(dummy_costs.begin(), dummy_costs.end(), 254.0);
//     if (max_cost_iterator == dummy_costs.end())
//     {
//       RCLCPP_INFO(logger_, "escape path is safe, publish velocity, %f", *max_cost_iterator);
//     }
//     else
//     {
//       RCLCPP_INFO(logger_, "escape path isn't safe, quit ");
//       return false;
//     }

//     return true;
//   }
//   else
//   {
//     RCLCPP_INFO(logger_, "???");
//   }

//   return true;
// }

double TrappedRecovery::scoreFootprint(std::vector<geometry_msgs::msg::Point> oriented_footprint)
{
  unsigned int x0, x1, y0, y1;
  double line_cost = 0.0;
  double footprint_cost = 0.0;

  for (size_t i = 0; i < oriented_footprint.size() - 1; ++i) {
    // get the cell coord of the first point
    if (!controller_costmap_->getCostmap()->worldToMap(
        oriented_footprint[i].x,
        oriented_footprint[i].y, x0, y0))
    {
      RCLCPP_ERROR(logger_, "scoreFootprint, get cell coord error.");
      return -1;  // indicates inner error
    }

    // get the cell coord of the second point
    if (!controller_costmap_->getCostmap()->worldToMap(
        oriented_footprint[i + 1].x,
        oriented_footprint[i + 1].y, x1, y1))
    {
      RCLCPP_ERROR(logger_, "scoreFootprint, get cell coord error..");
      return -1;
    }

    line_cost = lineCost(x0, x1, y0, y1);

    if (line_cost == -1) {
      return line_cost;  // -1 for LETHAL_OBSTACLE
    }
    footprint_cost = std::max(line_cost, footprint_cost);
  }

  // we also need to connect the first point in the footprint to the last point
  // get the cell coord of the last point
  if (!controller_costmap_->getCostmap()->worldToMap(
      oriented_footprint.back().x,
      oriented_footprint.back().y, x0, y0))
  {
    RCLCPP_ERROR(logger_, "scoreFootprint, get cell coord error...");
    return -1;
  }

  // get the cell coord of the first point
  if (!controller_costmap_->getCostmap()->worldToMap(
      oriented_footprint.front().x,
      oriented_footprint.front().y, x1, y1))
  {
    RCLCPP_ERROR(logger_, "scoreFootprint, get cell coord error....");
    return -1;
  }

  line_cost = lineCost(x0, x1, y0, y1);

  if (line_cost == -1) {
    return line_cost;  // -1 for LETHAL_OBSTACLE
  }
  footprint_cost = std::max(line_cost, footprint_cost);

  // if all line costs are legal... then we can return that the footprint is legal

  return footprint_cost;
}

void TrappedRecovery::transformFootprint(
  double x, double y, double theta, const std::vector<geometry_msgs::msg::Point> & footprint_spec,
  std::vector<geometry_msgs::msg::Point> & oriented_footprint)
{
  // build the oriented footprint at a given location
  oriented_footprint.clear();
  double cos_th = cos(theta);
  double sin_th = sin(theta);
  for (unsigned int i = 0; i < footprint_spec.size(); ++i) {
    geometry_msgs::msg::Point new_pt;
    new_pt.x = x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
    new_pt.y = y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
    oriented_footprint.push_back(new_pt);
  }
}

// void TrappedRecovery::initFootprint()
// {
//   geometry_msgs::msg::Point p;

//   p.x = 0.36, p.y = 0.18;
//   footprint_.push_back(p);

//   //   footprint_map_[Direction::Front_Left] = p;
//   //   cost_map_[Direction::Front_Left] = 0.0;

//   p.x = 0.36, p.y = 0;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Front_Middle] = p;
//   //   cost_map_[Direction::Front_Middle] = 0.0;

//   p.x = 0.36, p.y = -0.18;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Front_Right] = p;
//   //   cost_map_[Direction::Front_Right] = 0.0;

//   p.x = 0, p.y = -0.18;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Middle_Right] = p;
//   //   cost_map_[Direction::Middle_Right] = 0.0;

//   p.x = -0.415, p.y = -0.18;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Back_Right] = p;
//   //   cost_map_[Direction::Back_Right] = 0.0;

//   p.x = -0.415, p.y = 0;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Back_Middle] = p;
//   //   cost_map_[Direction::Back_Middle] = 0.0;

//   p.x = -0.415, p.y = 0.18;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Back_Left] = p;
//   //   cost_map_[Direction::Back_Left] = 0.0;

//   p.x = 0, p.y = 0.18;
//   footprint_.push_back(p);
//   //   footprint_map_[Direction::Middle_Left] = p;
//   //   cost_map_[Direction::Middle_Left] = 0.0;

//   //   for (auto f_it = footprint_map_.begin(), c_it = cost_map_.begin();
//             f_it != footprint_map_.end(); ++f_it,
//   ++c_it)
//   //   {
//   //     RCLCPP_INFO(logger_, "Footprint map [%d], x,y [%f, %f]", (int)(f_it->first),
//                                f_it->second.x,
//   f_it->second.y);
//   //     RCLCPP_INFO(logger_, "cost map [%d], cost[ %f ]", (int)(c_it->first), c_it->second);
//   //   }

//   // direction_map_;
//   direction_map_[0] = Eigen::Vector2d(1, 1);
//   direction_map_[1] = Eigen::Vector2d(1, 0);
//   direction_map_[2] = Eigen::Vector2d(1, -1);
//   direction_map_[3] = Eigen::Vector2d(0, -1);
//   direction_map_[4] = Eigen::Vector2d(-1, -1);
//   direction_map_[5] = Eigen::Vector2d(-1, 0);
//   direction_map_[6] = Eigen::Vector2d(-1, 1);
//   direction_map_[7] = Eigen::Vector2d(0, 1);
// }

double TrappedRecovery::lineCost(int x0, int x1, int y0, int y1)
{
  double line_cost = 0.0;  // save max cost
  double point_cost = -1.0;

  for (LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
    point_cost = getPointCost(line.getX(), line.getY());  // Score the current point

    if (point_cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
      point_cost = -1;
      return point_cost;
    }

    // no-info might be handled specially

    if (line_cost < point_cost) {
      line_cost = point_cost;
    }
  }

  return line_cost;
}

bool TrappedRecovery::transformPose(
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)
{
  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    // no block implementation
    tf_->transform(in_pose, out_pose, frame, tf2::durationFromSec(0.0));
    out_pose.header.frame_id = frame;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception in transformPose: %s", ex.what());
  }
  return false;
}

double TrappedRecovery::getPointCost(int x, int y)
{
  return controller_costmap_->getCostmap()->getCost(x, y);
}

// void TrappedRecovery::publishMarker(const std::vector<Eigen::Vector2d>& pts,
//                                     double angle, int i)
// {
//   visualization_msgs::msg::Marker marker;

//   marker.header.stamp = clock_->now();
//   marker.header.frame_id = controller_costmap_->getBaseFrameID();
//   marker.ns = "escape_direction";
//   marker.id = 0;
//   marker.action = visualization_msgs::msg::Marker::ADD;
//   marker.pose.position.x = marker.pose.position.y = 0;

//   marker.color.g = 1.0, marker.color.a = 1;

//   marker.lifetime.sec = 0;
//   marker.lifetime.nanosec = 0;

//   if (i)
//   {
//     marker.type = visualization_msgs::msg::Marker::POINTS;
//     marker.pose.orientation.w = 1.0;

//     for (size_t i = 0; i < pts.size(); ++i)
//     {
//       geometry_msgs::msg::Point p;
//       p.x = pts[i](0);
//       p.y = pts[i](1);
//       marker.points.push_back(p);
//     }

//     marker.scale.x = 0.1;
//     marker.scale.y = 0.1;
//     marker.scale.z = 0.1;
//   }
//   else
//   {
//     marker.type = visualization_msgs::msg::Marker::ARROW;

//     tf2::Quaternion q;
//     q.setRPY(0.f, 0.f, angle);

//     marker.pose.orientation = tf2::toMsg(q);

//     marker.scale.x = 0.5;
//     marker.scale.y = 0.1;
//     marker.scale.z = 0.1;
//   }

//   escapte_direction_pub_->publish(marker);
// }

void TrappedRecovery::ultrasonicCallback(const ception_msgs::msg::Around::SharedPtr msg)
{
  // RCLCPP_INFO(logger_, "ultrasonic in trapped, %f", msg->front_distance.range_info.range);
  last_ultrasonic_range_ = msg->front_distance.range_info;
}

bool TrappedRecovery::ultrasonicFrontFree()
{
  rclcpp::Time now = clock_->now();
  sensor_msgs::msg::Range temp = last_ultrasonic_range_;

  if (!isUltrasonicCurrent()) {
    return true;  // invalid ultrasonic data just ignore and go continue
  }
  if (temp.range < 0.3) {
    RCLCPP_INFO(logger_, "ultrasonicFrontFree, dangerous %f", temp.range);
    return false;
  }

  return true;
}

bool TrappedRecovery::isUltrasonicCurrent()
{
  rclcpp::Time now = clock_->now();
  sensor_msgs::msg::Range temp = last_ultrasonic_range_;
  double temp_stamp_t = rclcpp::Time(temp.header.stamp).seconds();

  if (temp_stamp_t == 0.0) {
    // no ultrasonic data received, ignore
    RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "ultrasonicFrontFree, no data: %f", temp.range);
    return false;
  }

  if (std::fabs(now.seconds() - temp_stamp_t) > 0.5) {
    // ultrasonic msg is not current, ignore
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 1000, "ultrasonicFrontFree, msg paused, [%.3f, %.3f]",
      now.seconds(), temp_stamp_t);
    return false;
  }

  return true;
}
}  // namespace move_base
