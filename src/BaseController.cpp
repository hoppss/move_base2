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
#include <ctime>
#include "move_base2/BaseController.hpp"

namespace move_base
{
static const double degree45 = 0.7854;
static const double degree20 = 0.349;
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

  nav2_util::declare_parameter_if_not_declared(
    node_, "dist_threshold", rclcpp::ParameterValue(0.1));
  node_->get_parameter("dist_threshold", dist_threshold_);
  nav2_util::declare_parameter_if_not_declared(
    node_, "theta_threshold", rclcpp::ParameterValue(0.1));
  node_->get_parameter("theta_threshold", theta_threshold_);

  backup_service_ = node_->create_service<std_srvs::srv::Trigger>("test_backup", 
                    std::bind(&BaseController::testBackup, this, std::placeholders::_1, std::placeholders::_2));

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

void BaseController::testBackup(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  auto r = request;
  response = response;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back up command");
  approachBackUp(0.5, 10.0);
}

bool BaseController::approachBackUp(const double dist, const double max_duration)
{
  bool result = true;
  //close the traj recorder while recovering. and check for traj's validation.
  trapped_->recorder_stop();
  std::deque<geometry_msgs::msg::PoseStamped>&& traj = trapped_->getTrajPoses();

  size_t k = 1;
  double length = 0.0;
  while (k < traj.size() && length < dist)
  {
    length += hypot(traj[k-1].pose.position.x - traj[k].pose.position.x,
                    traj[k-1].pose.position.y - traj[k].pose.position.y);
    k += 1;
  }
  
  geometry_msgs::msg::Twist twist;
  twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
  twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
  if(length < dist){
    RCLCPP_WARN(logger_, "there is no enough poses in traj historical container to support robot's back up recovery.");
    result = false;
  }else{
    //obtain the total poses for back up recovery, truncat the origin traj in trapped recovery.
    traj.resize(k);
    geometry_msgs::msg::PoseStamped target = traj.back(), odom_pose;
    double target_yaw = tf2::getYaw(target.pose.orientation);
    double det_x = 0.0, det_y = 0.0, det_t = 0.0, det_s = 0.0;

    time_t start_time, current_time;
    time(&start_time);time(&current_time);

    while(true && difftime(current_time, start_time) < max_duration){
      
      if(!getCurrentPose(odom_pose)){
        RCLCPP_WARN(logger_, "failed to get pose on odom frame during backup recovery.");
        result = false;
      }
      double current_yaw = tf2::getYaw(odom_pose.pose.orientation);                     
      // calcute the back up speed here.
      while(angles::shortest_angular_distance(tf2::getYaw(traj[0].pose.orientation), current_yaw) < degree20 &&
            hypot(odom_pose.pose.position.x - traj[0].pose.position.x, 
                  odom_pose.pose.position.y - traj[0].pose.position.y) < 0.1){
        traj.pop_front();
      }

      det_x = traj[0].pose.position.x - odom_pose.pose.position.x;
      det_y = traj[0].pose.position.y - odom_pose.pose.position.y;
      double rho = hypot(det_x, det_y);

      det_t = angles::normalize_angle(current_yaw - tf2::getYaw(traj[0].pose.orientation));
      twist.angular.z = std::copysign(std::min(abs(det_t), 0.3), det_t) * -1;
      if(abs(det_t) < degree10){
        twist.linear.x = std::min(abs(rho), 0.2) * -1.0;
      }

      // judgement of arrival target.
      det_s = hypot(odom_pose.pose.position.x - target.pose.position.x,
                           odom_pose.pose.position.y - target.pose.position.y);
      det_t = angles::shortest_angular_distance(current_yaw, target_yaw);
      
      if(det_s < dist_threshold_ && det_t < theta_threshold_){
        break;
      }

      publishVelocity(twist);
      time(&current_time);
    }
  }

  twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
  twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
  publishVelocity(twist);
  
  trapped_->truncatTrajFrontPoses();

  trapped_->recorder_start();
  return result;
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
