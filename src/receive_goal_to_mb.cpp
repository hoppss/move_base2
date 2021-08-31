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
#include <iomanip>
#include <string>
#include <memory>

#include "move_base2/receive_goal_to_mb.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/create_timer_ros.h"

namespace nav2_receive_goal
{
ReceiveGoalMb::ReceiveGoalMb()
: rclcpp::Node("receive_goal_to_mb"), start_tracking_(false)
//: nav2_util::LifecycleNode("receive_goal", "", true)
{
  declare_parameter("target_frame", rclcpp::ParameterValue("map"));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  get_parameter("target_frame", target_frame_);
  // get_parameter("transform_tolerance", transform_tolerance_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
    std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  src_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "tracking_pose", sub_qos,
    std::bind(&ReceiveGoalMb::srcPoseHandle, this, std::placeholders::_1));

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&ReceiveGoalMb::timerCallback, this));

  tracking_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
    "tracking_marker",
    10);

  // tar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tar_pose",
  // rclcpp::SystemDefaultsQoS());
  req_ = std::make_shared<automation_msgs::srv::NavigateToPose::Request>();
  navi_to_client_ = this->create_client<automation_msgs::srv::NavigateToPose>("NaviTo");
}

ReceiveGoalMb::~ReceiveGoalMb()
{
  timer_->cancel();
  navi_to_client_.reset();
  src_pose_sub_.reset();

  req_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  tracking_marker_pub_.reset();
}

// nav2_util::CallbackReturn
// ReceiveGoalMb::on_configure (const rclcpp_lifecycle::State & state) {
//
//    get_parameter("target_frame", target_frame_);
//    get_parameter("transform_tolerance", transform_tolerance_);
//
//    tf_buffer_ = std::make_shared< tf2_ros::Buffer>(rclcpp_node_->get_clock());
//    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
//        rclcpp_node_->get_node_base_interface(),
//        rclcpp_node_->get_node_timers_interface());
//    tf_buffer_->setCreateTimerInterface(timer_interface);
//    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//
//    src_pose_sub_ = create_subscription< geometry_msgs::msg::PoseStamped>(
//        "src_pose",
//        rclcpp::SystemDefaultsQoS(),
//        std::bind(&ReceiveGoalMb::srcPoseHandle, this, std::placeholders::_1));
//
//    tar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tar_pose",
//                    rclcpp::SystemDefaultsQoS());
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_activate (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_->on_activate();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_deactivate (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_->on_deactivate();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_cleanup (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_.reset();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_shutdown (const rclcpp_lifecycle::State & state) {
//    RCLCPP_INFO(get_logger(), "Shutting down");
//    return nav2_util::CallbackReturn::SUCCESS;
//}

void ReceiveGoalMb::srcPoseHandle(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "tracking_pose, in");
  geometry_msgs::msg::PoseStamped src_pose;
  src_pose = *msg;
  src_pose.header.stamp.sec = 0;
  src_pose.header.stamp.nanosec = 0;

  geometry_msgs::msg::PoseStamped tar_pose;
  if (false == transformPose(target_frame_, src_pose, tar_pose)) {
    RCLCPP_ERROR(this->get_logger(), "tracking_pose, transform pose failed");
    return;
  }

  std::lock_guard<std::mutex> guard(mutex_);
  goals_vec_.push_back(tar_pose);

  publishMarker(tar_pose);
}

bool ReceiveGoalMb::transformPose(
  const std::string & target_frame, const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    auto copy_in_pose = in_pose;
    copy_in_pose.header.stamp = rclcpp::Time();

    out_pose = tf_buffer_->transform(copy_in_pose, target_frame);
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      get_logger(), "ReceiveGoalMb: No Transform available Error looking up robot pose: %s\n",
      ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      get_logger(), "ReceiveGoalMb: Connectivity Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      get_logger(), "ReceiveGoalMb: Extrapolation Error looking up robot pose: %s\n", ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(get_logger(), "ReceiveGoalMb: Transform timeout with tolerance, %s", ex.what());
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "ReceiveGoalMb: Failed to transform %s", ex.what());
  }
  return false;
}

void ReceiveGoalMb::timerCallback()
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 2000, "receive_goal ...");

  if (goals_vec_.empty()) {
    return;
  }

  geometry_msgs::msg::PoseStamped goal;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal = goals_vec_.back();  // otain latest msg
    goals_vec_.clear();        // clear old data
  }

  if (nav2_util::geometry_utils::euclidean_distance(goal.pose, prev_pose_.pose) > 0.2) {
    //
    req_->planner_id = "DMP";
    req_->controller_id = "FollowPath";
    req_->goal = goal;

    prev_pose_ = goal;  // save last tracking goal
    publishMarker(prev_pose_, 100);

    // define async callback function
    auto response_received_callback =
      [this](rclcpp::Client<automation_msgs::srv::NavigateToPose>::SharedFuture result) {
        auto response = result.get();
        if (response->result == true && rclcpp::ok()) {
          RCLCPP_INFO(
            this->get_logger(), "client callback success %s.",
            response->description.c_str());
        } else {
          RCLCPP_ERROR(
            this->get_logger(), "client callback failed %s.",
            response->description.c_str());
        }
      };

    if (navi_to_client_->wait_for_service(std::chrono::milliseconds(100))) {
      auto future = navi_to_client_->async_send_request(req_, response_received_callback);
    }
  }
}

void ReceiveGoalMb::publishMarker(geometry_msgs::msg::PoseStamped & pose, int type)
{
  visualization_msgs::msg::Marker marker;
  marker.header = pose.header;
  marker.ns = "tracking";
  marker.id = 0;

  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = pose.pose;

  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  marker.color.g = 1.0, marker.color.a = 0.8;

  marker.lifetime.sec = 0;
  marker.lifetime.nanosec = 0;

  if (type != 1) {
    marker.id = 100;
    marker.color.g = 0;
    marker.color.r = 1;
  }

  tracking_marker_pub_->publish(marker);
}

}  // namespace nav2_receive_goal
