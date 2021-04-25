#ifndef MOVE_BASE_HPP
#define MOVE_BASE_HPP

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

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

#include "nav_2d_utils/odom_subscriber.hpp"
#include "nav_2d_utils/conversions.hpp"
#include "nav_2d_utils/tf_help.hpp"

#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "nav2_controller/plugins/simple_progress_checker.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"

#include "move_base2/state.hpp"
#include "move_base2/srv/navigate_to_pose.hpp"

namespace move_base
{
class MoveBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  MoveBase();
  ~MoveBase();

  using PlannerMap = std::unordered_map<std::string, nav2_core::GlobalPlanner::Ptr>;
  using ControllerMap = std::unordered_map<std::string, nav2_core::Controller::Ptr>;

  // planner
  nav_msgs::msg::Path getPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                              const std::string& planner_id);
  void publishPlan(const nav_msgs::msg::Path& path);

  bool getRobotPose(geometry_msgs::msg::PoseStamped& pose);

  // controller
  bool findControllerId(const std::string& c_name, std::string& current_controller);
  void computeControl();
  void setPlannerPath(const nav_msgs::msg::Path& path);
  void computeAndPublishVelocity();
  void updateGlobalPath(const nav_msgs::msg::Path& path);
  void publishVelocity(const geometry_msgs::msg::TwistStamped& velocity);
  void publishZeroVelocity();
  bool isGoalReached();

  inline double getThresholdedVelocity(double velocity, double threshold)
  {
    return (std::abs(velocity) > threshold) ? velocity : 0.0;
  }

  nav_2d_msgs::msg::Twist2D getThresholdedTwist(const nav_2d_msgs::msg::Twist2D& twist)
  {
    nav_2d_msgs::msg::Twist2D twist_thresh;
    twist_thresh.x = getThresholdedVelocity(twist.x, min_x_velocity_threshold_);
    twist_thresh.y = getThresholdedVelocity(twist.y, min_y_velocity_threshold_);
    twist_thresh.theta = getThresholdedVelocity(twist.theta, min_theta_velocity_threshold_);
    return twist_thresh;
  }

protected:
  // service server
  rclcpp::Service<move_base2::srv::NavigateToPose>::SharedPtr service_handle_;

  void handleService(const std::shared_ptr<move_base2::srv::NavigateToPose::Request> request,
                     std::shared_ptr<move_base2::srv::NavigateToPose::Response> response);
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

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
  nav2_costmap_2d::Costmap2D* global_costmap_;

  // controller
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> controller_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> controller_costmap_thread_;

  std::unique_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;

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
  double min_x_velocity_threshold_;
  double min_y_velocity_threshold_;
  double min_theta_velocity_threshold_;

  // Whether we've published the single controller warning yet
  geometry_msgs::msg::Pose end_pose_;

  // Clock for test funciton time cost
  rclcpp::Clock steady_clock_{ RCL_STEADY_TIME };

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace move_base

#endif  // MOVE_BASE_HPP