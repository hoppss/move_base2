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

#ifndef MOVE_BASE2__MOVEBASE_HPP_
#define MOVE_BASE2__MOVEBASE_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#include "rclcpp/rclcpp.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/progress_checker.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_core/exceptions.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_msgs/msg/costmap.hpp"

#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/odometry_utils.hpp"

#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"
#include "athena_utils/lifecycle_node.hpp"

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"

#include "automation_msgs/srv/nav_mode.hpp"
#include "automation_msgs/srv/navigate_to_pose.hpp"
#include "motion_msgs/msg/se3_velocity_cmd.hpp"
#include "motion_msgs/msg/frameid.hpp"

#include "move_base2/state.hpp"
#include "move_base2/request_info.hpp"
#include "move_base2/PointCost.hpp"
#include "move_base2/BaseController.hpp"
#include "move_base2/Reporter.hpp"
#include "move_base2/TrappedRecovery.hpp"

namespace move_base
{
class MoveBase : public athena_utils::LifecycleNode
{
public:
  MoveBase();
  ~MoveBase();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;

  // planner
  nav_msgs::msg::Path getPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::string & planner_id);
  void publishPlan(const nav_msgs::msg::Path & path);

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose);

  // controller
  bool findControllerId(const std::string & c_name, std::string & current_controller);
  void computeControl();
  void computeAndPublishVelocity();
  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  void publishZeroVelocity();
  bool isGoalReached();

  inline double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  geometry_msgs::msg::Twist getThresholdedTwist(const geometry_msgs::msg::Twist & twist)
  {
    geometry_msgs::msg::Twist twist_thresh;
    twist_thresh.linear.x = getThresholdedVelocity(twist.linear.x, min_x_velocity_threshold_);
    twist_thresh.linear.y = getThresholdedVelocity(twist.linear.y, min_y_velocity_threshold_);
    twist_thresh.angular.z = getThresholdedVelocity(twist.angular.z, min_theta_velocity_threshold_);
    return twist_thresh;
  }

  void loop();  // loop control

protected:
  // lifecycle flag, aferactive is ready
  NavState state_;

  std::atomic_bool is_cancel_;

  bool is_shutdown_;  // flag for ctrl_c signal. make sure plan_thread_ exit

  // set_mode
  int navi_mode_;

  // service server
  rclcpp::Service<automation_msgs::srv::NavigateToPose>::SharedPtr service_handle_;

  void handleService(
    const std::shared_ptr<automation_msgs::srv::NavigateToPose::Request> request,
    std::shared_ptr<automation_msgs::srv::NavigateToPose::Response> response);

  std::queue<requestInfo> goals_queue_;

  requestInfo current_request_;

  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> gp_loader_;
  std::vector<std::string> default_planner_ids_;
  std::vector<std::string> default_planner_types_;

  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_publisher_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;
  nav2_costmap_2d::Costmap2D * global_costmap_;

  // Global, function, status
  nav_msgs::msg::Path last_global_plan_;
  rclcpp::Time last_nofity_plan_time_;
  rclcpp::Time last_valid_plan_time_;
  std::atomic_bool new_global_plan_;
  int failed_control_cnt_;

  void planThread();
  std::shared_ptr<std::thread> plan_thread_;
  std::mutex planner_mutex_;
  std::condition_variable planner_cond_;
  bool run_planner_;

  // controller
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> controller_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> controller_costmap_thread_;

  // controller, function, status
  rclcpp::Time last_valid_control_time_;

  std::unique_ptr<nav2_util::OdomSmoother> odom_sub_;
  std::string default_odom_topic_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<motion_msgs::msg::SE3VelocityCMD>::SharedPtr
    body_cmd_publisher_;

  // controller, Progress Checker Plugin
  pluginlib::ClassLoader<nav2_core::ProgressChecker> progress_checker_loader_;
  nav2_core::ProgressChecker::Ptr progress_checker_;
  std::string default_progress_checker_id_;
  std::string default_progress_checker_type_;
  std::string progress_checker_id_;
  std::string progress_checker_type_;

  // controller, goal_checker
  pluginlib::ClassLoader<nav2_core::GoalChecker> goal_checker_loader_;
  nav2_core::GoalChecker::Ptr goal_checker_;
  std::string default_goal_checker_id_;
  std::string default_goal_checker_type_;
  std::string goal_checker_id_;
  std::string goal_checker_type_;

  // Controller Plugins
  pluginlib::ClassLoader<nav2_core::Controller> lp_loader_;
  ControllerMap controllers_;
  std::vector<std::string> default_controller_ids_;
  std::vector<std::string> default_controller_types_;
  std::vector<std::string> controller_ids_;
  std::vector<std::string> controller_types_;
  std::string controller_ids_concat_, current_controller_;

  double controller_frequency_;
  int period_;  // 100 multiper, in ms
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::Pose end_pose_;

  // Clock for test funciton time cost
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  std::shared_ptr<std::thread> spin_thread_;
  void spinThread();

public:
  void resetState();

  rclcpp::Service<automation_msgs::srv::NavMode>::SharedPtr get_mode_server_;
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr param_client_;

  void getModeCallback(
    const std::shared_ptr<automation_msgs::srv::NavMode::Request> req,
    std::shared_ptr<automation_msgs::srv::NavMode::Response> res);

  bool setControllerTrackingMode(bool enable);

  // ns
  std::string ns_;

  // obstacle detect
  std::shared_ptr<PointCost> point_cost_;
  std::shared_ptr<TrappedRecovery> trapped_recovery_;

  bool transformPose(
    const std::string & target_frame,
    const geometry_msgs::msg::PoseStamped & in_pose,
    geometry_msgs::msg::PoseStamped & out_pose);

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr tracking_pose_sub_;
  void trackingPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    tracking_marker_pub_;
  void publishMarker(geometry_msgs::msg::PoseStamped & pose, int type = 1);
  geometry_msgs::msg::PoseStamped pre_tracking_pose_;
  void updateTrackingGoal();

  // tracking recovery
  geometry_msgs::msg::PoseStamped last_tracking_pose_in_camera_;
  std::shared_ptr<BaseController> base_controller_;

  // status_reporter
  std::shared_ptr<Reporter> reporter_;
};

}  // namespace move_base

#endif  // MOVE_BASE2__MOVEBASE_HPP_
