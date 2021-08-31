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

#ifndef MOVE_BASE2__RECEIVE_GOAL_TO_MB_HPP_
#define MOVE_BASE2__RECEIVE_GOAL_TO_MB_HPP_

#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "nav2_util/lifecycle_node.hpp"

#include "automation_msgs/srv/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_receive_goal
{
// class ReceiveGoalMb : public nav2_util::LifecycleNode
class ReceiveGoalMb : public rclcpp::Node
{
public:
  ReceiveGoalMb();

  ~ReceiveGoalMb();

  // nav2_util::CallbackReturn on_configure (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_activate (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_deactivate (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_cleanup (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_shutdown (const rclcpp_lifecycle::State & state) override;

private:
  void srcPoseHandle(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool transformPose(
    const std::string & target_frame, const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose);

  // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tar_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr src_pose_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;


  bool start_tracking_;
  geometry_msgs::msg::PoseStamped prev_pose_;

  rclcpp::Client<automation_msgs::srv::NavigateToPose>::SharedPtr navi_to_client_;
  std::shared_ptr<automation_msgs::srv::NavigateToPose::Request> req_;

  std::mutex mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_vec_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_marker_pub_;
  void publishMarker(geometry_msgs::msg::PoseStamped & pose, int type = 1);
};

}  // namespace nav2_receive_goal

#endif  // MOVE_BASE2__RECEIVE_GOAL_TO_MB_HPP_
