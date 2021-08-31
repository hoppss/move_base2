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
#include <string>
#include <utility>
#include <vector>
#include "move_base2/BaseController.hpp"

namespace move_base
{
static const double degree45 = 0.7854;
static const double degree10 = 0.1745;
static const double degree5 = 0.0872;

BaseController::BaseController()
: name_{"base_controller"}, logger_(rclcpp::get_logger("move_base_basecontroller"))
{
}

BaseController::~BaseController()
{
}

void BaseController::initialize(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent,
  const std::shared_ptr<tf2_ros::Buffer> & tf,
  const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher,
  const rclcpp_lifecycle::LifecyclePublisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr
  body_cmd_publisher,
  const std::shared_ptr<move_base::TrappedRecovery> & trapped_ptr)
{
  node_ = parent;

  clock_ = node_->get_clock();

  tf_ = tf;
  vel_pub_ = vel_publisher;
  body_cmd_pub_ = body_cmd_publisher;
  trapped_ = trapped_ptr;
}

bool BaseController::transformPose(
  const std::string & target_frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose)
{
  if (in_pose.header.frame_id == target_frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    auto copy_in_pose = in_pose;
    copy_in_pose.header.stamp = rclcpp::Time();
    out_pose = tf_->transform(copy_in_pose, target_frame);
    return true;
  } catch (tf2::LookupException & ex) {
    RCLCPP_ERROR(
      logger_, "transformPose: No Transform available Error looking up robot pose: %s\n",
      ex.what());
  } catch (tf2::ConnectivityException & ex) {
    RCLCPP_ERROR(
      logger_, "transformPose: Connectivity Error looking up robot pose: %s\n",
      ex.what());
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(
      logger_, "transformPose: Extrapolation Error looking up robot pose: %s\n",
      ex.what());
  } catch (tf2::TimeoutException & ex) {
    RCLCPP_ERROR(logger_, "transformPose: Transform timeout with tolerance%s", ex.what());
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "transformPose: Failed to transform");
  }

  return false;
}

bool BaseController::getCurrentPose(geometry_msgs::msg::PoseStamped & odom_pose)
{
  geometry_msgs::msg::PoseStamped p_in_b;

  tf2::toMsg(tf2::Transform::getIdentity(), p_in_b.pose);

  p_in_b.header.frame_id = "base_footprint";
  p_in_b.header.stamp = rclcpp::Time();

  odom_pose.header.frame_id = "odom";
  odom_pose.header.stamp = rclcpp::Time();

  if (!transformPose(odom_pose.header.frame_id, p_in_b, odom_pose)) {
    RCLCPP_ERROR(logger_, "BaseController, getCurrentPose failed");
    return false;
  }
  return true;
}

bool BaseController::approachOnlyRotate(const geometry_msgs::msg::PoseStamped & target)
{
  // 0. zero cmd
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.0;
  command.angular.z = 0.0;

  // firstly, collsion check
  geometry_msgs::msg::PoseStamped current_pose, copy_target;
  copy_target = target;
  std::vector<geometry_msgs::msg::PoseStamped> result;
  if (getCurrentPose(current_pose)) {
    interpolateToTarget(current_pose, copy_target, result);

    if (!result.empty()) {
      for (size_t i = 0; i < result.size(); ++i) {
        if (trapped_->isTrappedInPose(
            result[i].pose.position.x, result[i].pose.position.y,
            tf2::getYaw(result[i].pose.orientation)))
        {
          RCLCPP_ERROR(logger_, "in-place rotate collision check illegal");
          publishVelocity(command);
          return false;
        }
      }
    }
  } else {
    publishVelocity(command);
    return false;
  }

  // approach
  geometry_msgs::msg::PoseStamped p_in_b;

  double theta = 0.0;
  bool goal_reached = false;

  if (transformPose("base_footprint", target, p_in_b)) {
    theta = tf2::getYaw(p_in_b.pose.orientation);
    if (std::fabs(theta) > degree5) {
      command.angular.z = theta;
    } else {
      command.angular.z = 0.0;
      goal_reached = true;
    }

    if (theta > degree45) {
      command.angular.z = degree45;
    }
    if (theta < -degree45) {
      command.angular.z = -degree45;
    }

    publishVelocity(command);

    if (goal_reached) {
      RCLCPP_INFO(logger_, "rotate finish");
      return true;
    } else {
      return false;
    }
  }

  publishVelocity(command);  // zero if error occurs
  return false;
}

bool BaseController::interpolateToTarget(
  geometry_msgs::msg::PoseStamped & start,
  geometry_msgs::msg::PoseStamped & goal,
  std::vector<geometry_msgs::msg::PoseStamped> & result_v)
{
  result_v.clear();

  tf2::Quaternion q_start, q_goal;
  tf2::fromMsg(start.pose.orientation, q_start);
  tf2::fromMsg(goal.pose.orientation, q_goal);

  geometry_msgs::msg::PoseStamped p = start;

  double delta_angle =
    angles::shortest_angular_distance(tf2::getYaw(q_goal), tf2::getYaw(q_start));    // goal-start
  double step_length = 15.0 / 180.0 * M_PI;  // every 15° collision check

  int steps = std::floor(std::abs(delta_angle) / step_length);

  result_v.push_back(start);  // push current pose firstly

  if (steps > 0) {
    for (int i = 1; i <= steps; ++i) {
      tf2::Quaternion temp_q = q_start.slerp(q_goal, i * 1.0 / steps);
      p.pose.orientation = tf2::toMsg(temp_q);
      result_v.push_back(p);  // internal interpolate point
    }
  }

  result_v.push_back(goal);  // push goal pose finally

  // show result to debug
  // std::cout << "interpolation from [" << tf2::getYaw(start.pose.orientation) << " : "
  //           << tf2::getYaw(goal.pose.orientation) << "], delta_angle is " << delta_angle
  //           << ", steps: " << steps << ", sizes %ld" << result_v.size() << std::endl;

  // for (size_t i = 0; i < result_v.size(); ++i)
  // {
  //   std::cout << "result " << i << " " << tf2::getYaw(result_v[i].pose.orientation) << " -> "
  //             << tf2::getYaw(result_v[i].pose.orientation) / M_PI * 180.0 << std::endl;
  // }

  std::cout << std::endl;

  return true;
}

void BaseController::publishVelocity(const geometry_msgs::msg::Twist & command)
{
  geometry_msgs::msg::Twist temp = command;

  vel_pub_->publish(temp);  // twist cmd

  motion_msgs::msg::SE3VelocityCMD cmd;
  cmd.sourceid = motion_msgs::msg::SE3VelocityCMD::NAVIGATOR;
  cmd.velocity.frameid.id = motion_msgs::msg::Frameid::BODY_FRAME;
  cmd.velocity.timestamp = clock_->now();
  cmd.velocity.linear_x = temp.linear.x;
  cmd.velocity.linear_y = temp.linear.y;
  cmd.velocity.linear_z = temp.linear.z;
  cmd.velocity.angular_x = temp.angular.x;
  cmd.velocity.angular_y = temp.angular.y;
  cmd.velocity.angular_z = temp.angular.z;
  body_cmd_pub_->publish(std::move(cmd));  // se3 cmd

  std::this_thread::sleep_for(std::chrono::milliseconds(100));  // rate-control
}

}  // namespace move_base
