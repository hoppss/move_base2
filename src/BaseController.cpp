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
#include <memory>
#include <string>
#include <utility>
#include "move_base2/BaseController.h"

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
  body_cmd_publisher)
{
  node_ = parent;

  clock_ = node_->get_clock();

  tf_ = tf;
  vel_pub_ = vel_publisher;
  body_cmd_pub_ = body_cmd_publisher;
}

bool BaseController::transformPose(
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
  geometry_msgs::msg::PoseStamped p_in_b;

  double theta = 0.0;
  bool goal_reached = false;

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.0;
  command.angular.z = 0.0;

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

    vel_pub_->publish(command);

    {
      motion_msgs::msg::SE3VelocityCMD cmd;
      cmd.sourceid = motion_msgs::msg::SE3VelocityCMD::NAVIGATOR;
      cmd.velocity.frameid.id = motion_msgs::msg::Frameid::BODY_FRAME;
      cmd.velocity.timestamp = clock_->now();
      cmd.velocity.linear_x = command.linear.x;
      cmd.velocity.linear_y = command.linear.y;
      cmd.velocity.linear_z = command.linear.z;
      cmd.velocity.angular_x = command.angular.x;
      cmd.velocity.angular_y = command.angular.y;
      cmd.velocity.angular_z = command.angular.z;
      body_cmd_pub_->publish(std::move(cmd));
    }

    if (goal_reached) {
      RCLCPP_INFO(logger_, "rotate finish");
      return true;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      return false;
    }
  }

  vel_pub_->publish(command);
  {
    motion_msgs::msg::SE3VelocityCMD cmd;
    cmd.sourceid = motion_msgs::msg::SE3VelocityCMD::NAVIGATOR;
    cmd.velocity.frameid.id = motion_msgs::msg::Frameid::BODY_FRAME;
    cmd.velocity.timestamp = clock_->now();
    cmd.velocity.linear_x = command.linear.x;
    cmd.velocity.linear_y = command.linear.y;
    cmd.velocity.linear_z = command.linear.z;
    cmd.velocity.angular_x = command.angular.x;
    cmd.velocity.angular_y = command.angular.y;
    cmd.velocity.angular_z = command.angular.z;
    body_cmd_pub_->publish(std::move(cmd));
  }
  return false;
}

bool BaseController::rotate(double angle)
{
  //   geometry_msgs::msg::PoseStamped p_in_b, p_in_o;

  //   tf2::toMsg(tf2::Transform::getIdentity(), p_in_b.pose);
  //   tf2::toMsg(tf2::Transform::getIdentity(), p_in_o.pose);

  //   p_in_b.header.frame_id = "base_footprint";

  //   tf2::Quaternion q;
  //   q.setRPY(0.0, 0.0, angle);
  //   p_in_b.pose.orientation = tf2::toMsg(q);

  //   try
  //   {
  //     // tf_->waitForTransform("odom", "base_footprint", tf2::durationFromSec(1.0));
  //     p_in_o = tf_->transform(p_in_b, "odom", tf2::durationFromSec(0.0));
  //   }
  //   catch (tf2::LookupException& ex)
  //   {
  //     RCLCPP_ERROR(logger_,
  //                  "No Transform available Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::ConnectivityException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Connectivity Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::ExtrapolationException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Extrapolation Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::TimeoutException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Transform timeout with tolerance");
  //     return false;
  //   }
  //   catch (tf2::TransformException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Failed to transform from base to odom");
  //     return false;
  //   }

  double speed = M_PI_4;
  int cnt = static_cast<int>(std::ceil(std::fabs(angle) / speed * 15));

  while (cnt > 0) {
    RCLCPP_INFO(logger_, "rotating cnt %d", cnt);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = std::copysign(speed, angle);

    vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cnt--;
  }

  RCLCPP_INFO(logger_, "rotate finish");
  return true;
}

}  // namespace move_base
