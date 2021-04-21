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
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_core/global_planner.hpp"

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

  /**
   * @brief Method to get plan from the desired plugin
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  nav_msgs::msg::Path getPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                              const std::string& planner_id);

protected:
  // service server
  rclcpp::Service<move_base2::srv::NavigateToPose>::SharedPtr service_;

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
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;

  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;

  double max_planner_duration_;
  std::string planner_ids_concat_;

  // controller

  // // Clock
  // rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // Global Costmap
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
  std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;
  nav2_costmap_2d::Costmap2D* global_costmap_;
};

}  // namespace move_base

#endif  // MOVE_BASE_HPP
