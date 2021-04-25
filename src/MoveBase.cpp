#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "move_base2/MoveBase.hpp"

using namespace std::chrono_literals;

namespace move_base
{
MoveBase::MoveBase()
  : rclcpp_lifecycle::LifecycleNode("move_base_node")

  , gp_loader_("nav2_core", "nav2_core::GlobalPlanner")
  , default_planner_ids_{ "GridBased" }
  , default_planner_types_{ "nav2_navfn_planner/NavfnPlanner" }
  , global_costmap_(nullptr)

  , progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker")
  , default_progress_checker_id_{ "progress_checker" }
  , default_progress_checker_type_{ "nav2_controller::SimpleProgressChecker" }

  , goal_checker_loader_("nav2_core", "nav2_core::GoalChecker")
  , default_goal_checker_id_{ "goal_checker" }
  , default_goal_checker_type_{ "nav2_controller::SimpleGoalChecker" }

  , lp_loader_("nav2_core", "nav2_core::Controller")
  , default_controller_ids_{ "FollowPath" }
  , default_controller_types_{ "dwb_core::DWBLocalPlanner" }
{
  // setup planner+global_costmap
  RCLCPP_INFO(get_logger(), "Creating Planner");

  for (size_t i = 0; i < default_planner_ids_.size(); ++i)
  {
    RCLCPP_INFO(get_logger(), "default_planner_ids_[%d]: %s", i, default_planner_ids_[i].c_str());  // GridBased
    RCLCPP_INFO(get_logger(), "default_planner_types_[%d] : %s", i,
                default_planner_types_[i].c_str());  // nav2_navfn_planner/NavfnPlanner
  }

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_planner_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);

  if (planner_ids_ == default_planner_ids_)
  {
    for (size_t i = 0; i < default_planner_ids_.size(); ++i)
    {
      declare_parameter(default_planner_ids_[i] + ".plugin", default_planner_types_[i]);
      RCLCPP_INFO(get_logger(), "default_planner_types[%i] : %s", i,
                  default_planner_types_[i].c_str());  // nav2_navfn_planner/NavfnPlanner
    }
  }

  // Setup the global costmap
  RCLCPP_INFO(get_logger(), "what is namespace: %s", get_namespace());
  global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{ get_namespace() }, "global_costmap");

  // Launch a thread to run the costmap node, it's singleThreadExecutor
  global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);

  // Setup controller + local_costmap
  RCLCPP_INFO(get_logger(), "Creating Controller");
  for (size_t i = 0; i < default_controller_ids_.size(); ++i)
  {
    RCLCPP_INFO(get_logger(), "default_controller_ids_[%d]: %s", i, default_controller_ids_[i].c_str());  // FollowPath
    RCLCPP_INFO(get_logger(), "default_controller_types_[%d] : %s", i,
                default_controller_types_[i].c_str());  // dwb_core::DWBLocalPlanner
  }

  declare_parameter("controller_frequency", 20.0);

  declare_parameter("progress_checker_plugin", default_progress_checker_id_);
  declare_parameter("goal_checker_plugin", default_goal_checker_id_);
  declare_parameter("controller_plugins", default_controller_ids_);

  declare_parameter("min_x_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_y_velocity_threshold", rclcpp::ParameterValue(0.0001));
  declare_parameter("min_theta_velocity_threshold", rclcpp::ParameterValue(0.0001));

  // The costmap node is used in the implementation of the controller
  controller_costmap_ros_ =
      std::make_shared<nav2_costmap_2d::Costmap2DROS>("local_costmap", std::string{ get_namespace() }, "local_costmap");

  // Launch a thread to run the costmap node
  controller_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(controller_costmap_ros_);
}

MoveBase::~MoveBase()
{
  RCLCPP_INFO(get_logger(), "Destroying");
  planners_.clear();
  global_costmap_thread_.reset();
}

nav2_util::CallbackReturn MoveBase::on_configure(const rclcpp_lifecycle::State& state)
{
  // configure planner
  RCLCPP_INFO(get_logger(), "Configuring planner interface");

  global_costmap_ros_->on_configure(state);
  global_costmap_ = global_costmap_ros_->getCostmap();

  RCLCPP_DEBUG(get_logger(), "Costmap size: %d,%d", global_costmap_->getSizeInCellsX(),
               global_costmap_->getSizeInCellsY());

  tf_ = global_costmap_ros_->getTfBuffer();

  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++)
  {
    try
    {
      planner_types_[i] = nav2_util::get_plugin_type_param(node, planner_ids_[i]);
      nav2_core::GlobalPlanner::Ptr planner = gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(get_logger(), "Created global planner plugin %s of type %s", planner_ids_[i].c_str(),
                  planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i], tf_, global_costmap_ros_);
      planners_.insert({ planner_ids_[i], planner });
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(), "Failed to create global planner. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++)
  {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(get_logger(), "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0)
  {
    max_planner_duration_ = 1 / expected_planner_frequency;
  }
  else
  {
    RCLCPP_WARN(get_logger(),
                "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
                " than 0.0 to turn on duration overrrun warning messages",
                expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Create global path publisher
  plan_publisher_ = create_publisher<nav_msgs::msg::Path>("plan", 1);

  // configure controllers
  RCLCPP_INFO(get_logger(), "Configuring controller interface");

  get_parameter("progress_checker_plugin", progress_checker_id_);
  if (progress_checker_id_ == default_progress_checker_id_)
  {
    nav2_util::declare_parameter_if_not_declared(node, default_progress_checker_id_ + ".plugin",
                                                 rclcpp::ParameterValue(default_progress_checker_type_));
  }

  get_parameter("goal_checker_plugin", goal_checker_id_);
  if (goal_checker_id_ == default_goal_checker_id_)
  {
    nav2_util::declare_parameter_if_not_declared(node, default_goal_checker_id_ + ".plugin",
                                                 rclcpp::ParameterValue(default_goal_checker_type_));
  }

  get_parameter("controller_plugins", controller_ids_);
  if (controller_ids_ == default_controller_ids_)
  {
    for (size_t i = 0; i < default_controller_ids_.size(); ++i)
    {
      nav2_util::declare_parameter_if_not_declared(node, default_controller_ids_[i] + ".plugin",
                                                   rclcpp::ParameterValue(default_controller_types_[i]));
    }
  }

  controller_types_.resize(controller_ids_.size());

  get_parameter("controller_frequency", controller_frequency_);
  get_parameter("min_x_velocity_threshold", min_x_velocity_threshold_);
  get_parameter("min_y_velocity_threshold", min_y_velocity_threshold_);
  get_parameter("min_theta_velocity_threshold", min_theta_velocity_threshold_);
  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz", controller_frequency_);

  controller_costmap_ros_->on_configure(state);

  try
  {
    progress_checker_type_ = nav2_util::get_plugin_type_param(node, progress_checker_id_);
    progress_checker_ = progress_checker_loader_.createUniqueInstance(progress_checker_type_);
    RCLCPP_INFO(get_logger(), "Created progress_checker : %s of type %s", progress_checker_id_.c_str(),
                progress_checker_type_.c_str());
    progress_checker_->initialize(node, progress_checker_id_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(), "Failed to create progress_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  try
  {
    goal_checker_type_ = nav2_util::get_plugin_type_param(node, goal_checker_id_);
    goal_checker_ = goal_checker_loader_.createUniqueInstance(goal_checker_type_);
    RCLCPP_INFO(get_logger(), "Created goal_checker : %s of type %s", goal_checker_id_.c_str(),
                goal_checker_type_.c_str());
    goal_checker_->initialize(node, goal_checker_id_);
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    RCLCPP_FATAL(get_logger(), "Failed to create goal_checker. Exception: %s", ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  for (size_t i = 0; i != controller_ids_.size(); i++)
  {
    try
    {
      controller_types_[i] = nav2_util::get_plugin_type_param(node, controller_ids_[i]);
      nav2_core::Controller::Ptr controller = lp_loader_.createUniqueInstance(controller_types_[i]);
      RCLCPP_INFO(get_logger(), "Created controller : %s of type %s", controller_ids_[i].c_str(),
                  controller_types_[i].c_str());
      controller->configure(node, controller_ids_[i], controller_costmap_ros_->getTfBuffer(), controller_costmap_ros_);
      controllers_.insert({ controller_ids_[i], controller });
    }
    catch (const pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(get_logger(), "Failed to create controller. Exception: %s", ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != controller_ids_.size(); i++)
  {
    controller_ids_concat_ += controller_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(get_logger(), "Controller Server has %s controllers available.", controller_ids_concat_.c_str());

  odom_sub_ = std::make_unique<nav_2d_utils::OdomSubscriber>(node);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // Finally, Create the action servers for path planning to a pose and through poses
  service_handle_ = this->create_service<move_base2::srv::NavigateToPose>(
      "/NaviTo", std::bind(&MoveBase::handleService, this, std::placeholders::_1, std::placeholders::_2));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  global_costmap_ros_->on_activate(state);
  plan_publisher_->on_activate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->activate();
  }

  controller_costmap_ros_->on_activate(state);
  ControllerMap::iterator c_it;
  for (c_it = controllers_.begin(); c_it != controllers_.end(); ++c_it)
  {
    c_it->second->activate();
  }
  vel_publisher_->on_activate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  global_costmap_ros_->on_deactivate(state);
  plan_publisher_->on_deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->deactivate();
  }

  // controllers
  ControllerMap::iterator c_it;
  for (c_it = controllers_.begin(); c_it != controllers_.end(); ++c_it)
  {
    c_it->second->deactivate();
  }
  controller_costmap_ros_->on_deactivate(state);

  publishZeroVelocity();
  vel_publisher_->on_deactivate();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_cleanup(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  plan_publisher_.reset();

  tf_.reset();
  global_costmap_ros_->on_cleanup(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->cleanup();
  }
  planners_.clear();
  global_costmap_ = nullptr;

  // controllers
  ControllerMap::iterator c_it;
  for (c_it = controllers_.begin(); c_it != controllers_.end(); ++c_it)
  {
    c_it->second->cleanup();
  }
  controllers_.clear();
  controller_costmap_ros_->on_cleanup(state);

  // Release any allocated resources
  odom_sub_.reset();
  vel_publisher_.reset();
  goal_checker_->reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_shutdown(const rclcpp_lifecycle::State&)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MoveBase::handleService(const std::shared_ptr<move_base2::srv::NavigateToPose::Request> request,
                             std::shared_ptr<move_base2::srv::NavigateToPose::Response> response)
{
  //
  RCLCPP_INFO(get_logger(), "move_base2_service callback");

  RCLCPP_INFO(get_logger(), "1. getcurrentpose");

  geometry_msgs::msg::PoseStamped start;
  if (!global_costmap_ros_->getRobotPose(start))
  {
    RCLCPP_ERROR(get_logger(), "1. getcurrentpose failed");
    response->result = move_base2::srv::NavigateToPose::Response::FAILTURE;
    return;
  }
  RCLCPP_INFO(get_logger(), "2. makeplan");
  geometry_msgs::msg::PoseStamped goal = request->goal;
  nav_msgs::msg::Path path = getPlan(start, goal, request->planner_id);

  if (path.poses.size() == 0)
  {
    RCLCPP_WARN(get_logger(),
                "Planning algorithm %s failed to generate a valid"
                " path to (%.2f, %.2f)",
                request->planner_id.c_str(), goal.pose.position.x, goal.pose.position.y);
    response->result = move_base2::srv::NavigateToPose::Response::FAILTURE;
    return;
  }

  RCLCPP_INFO(get_logger(), "Found valid path of size %lu to (%.2f, %.2f)", path.poses.size(), goal.pose.position.x,
              goal.pose.position.y);

  // Publish the plan for visualization purposes
  RCLCPP_INFO(get_logger(), "3. publish path");

  publishPlan(path);
}

nav_msgs::msg::Path MoveBase::getPlan(const geometry_msgs::msg::PoseStamped& start,
                                      const geometry_msgs::msg::PoseStamped& goal, const std::string& planner_id)
{
  RCLCPP_INFO(get_logger(),
              "Attempting to a find path from (%.2f, %.2f) to "
              "(%.2f, %.2f).",
              start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

  if (planners_.find(planner_id) != planners_.end())
  {
    return planners_[planner_id]->createPlan(start, goal);
  }
  else
  {
    if (planners_.size() == 1 && planner_id.empty())
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "No planners specified in action call. "
                       "Server will use only plugin %s in server."
                       " This warning will appear once.",
                       planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->createPlan(start, goal);
    }
    else
    {
      RCLCPP_ERROR(get_logger(),
                   "planner %s is not a valid planner. "
                   "Planner names are: %s",
                   planner_id.c_str(), planner_ids_concat_.c_str());
    }
  }

  return nav_msgs::msg::Path();
}

void MoveBase::publishPlan(const nav_msgs::msg::Path& path)
{
  auto msg = std::make_unique<nav_msgs::msg::Path>(path);
  if (plan_publisher_->is_activated() /*&& plan_publisher_->get_subscription_count() > 0*/)
  {
    plan_publisher_->publish(std::move(msg));
  }
}

// Following is controller's function
bool MoveBase::findControllerId(const std::string& c_name, std::string& current_controller)
{
  if (controllers_.find(c_name) == controllers_.end())
  {
    if (controllers_.size() == 1 && c_name.empty())
    {
      RCLCPP_WARN_ONCE(get_logger(),
                       "No controller was specified in action call."
                       " Server will use only plugin loaded %s. "
                       "This warning will appear once.",
                       controller_ids_concat_.c_str());
      current_controller = controllers_.begin()->first;
    }
    else
    {
      RCLCPP_ERROR(get_logger(),
                   "FollowPath called with controller name %s, "
                   "which does not exist. Available controllers are %s.",
                   c_name.c_str(), controller_ids_concat_.c_str());
      return false;
    }
  }
  else
  {
    RCLCPP_DEBUG(get_logger(), "Selected controller: %s.", c_name.c_str());
    current_controller = c_name;
  }

  return true;
}

void MoveBase::computeControl()
{
  RCLCPP_INFO(get_logger(), "Received a goal, begin computing control effort.");

  try
  {
    // setPlannerPath(last_valid_plan_);
    progress_checker_->reset();

    rclcpp::WallRate loop_rate(controller_frequency_);
    while (rclcpp::ok())
    {
      if (1)
      {
        RCLCPP_INFO(get_logger(), "Goal was canceled. Stopping the robot.");
        // action_server_->terminate_all();
        publishZeroVelocity();
        return;
      }

      // updateGlobalPath();

      computeAndPublishVelocity();

      if (isGoalReached())
      {
        RCLCPP_INFO(get_logger(), "Reached the goal!");
        break;
      }

      if (!loop_rate.sleep())
      {
        RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz", controller_frequency_);
      }
    }
  }
  catch (nav2_core::PlannerException& e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    // action_server_->terminate_current();
    return;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");

  publishZeroVelocity();

  // TODO(orduno) #861 Handle a pending preemption and set controller name
  // action_server_->succeeded_current();
}

void MoveBase::setPlannerPath(const nav_msgs::msg::Path& path)
{
  RCLCPP_INFO(get_logger(), "Providing path to the controller %s", current_controller_.c_str());
  if (path.poses.empty())
  {
    throw nav2_core::PlannerException("Invalid path, Path is empty.");
  }
  controllers_[current_controller_]->setPlan(path);

  auto end_pose = path.poses.back();
  end_pose.header.frame_id = path.header.frame_id;
  rclcpp::Duration tolerance(1e9);
  nav_2d_utils::transformPose(tf_, controller_costmap_ros_->getGlobalFrameID(), end_pose, end_pose, tolerance);
  goal_checker_->reset();

  RCLCPP_DEBUG(get_logger(), "Path end point is (%.2f, %.2f)", end_pose.pose.position.x, end_pose.pose.position.y);
  end_pose_ = end_pose.pose;
}

void MoveBase::computeAndPublishVelocity()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose))
  {
    throw nav2_core::PlannerException("Failed to obtain robot pose");
  }

  if (!progress_checker_->check(pose))
  {
    throw nav2_core::PlannerException("Failed to make progress");
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());

  auto cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(pose, nav_2d_utils::twist2Dto3D(twist));

  // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);
  // feedback->distance_to_goal = nav2_util::geometry_utils::euclidean_distance(end_pose_, pose.pose);
  // action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
}

void MoveBase::updateGlobalPath(const nav_msgs::msg::Path& path)
{
  RCLCPP_INFO(get_logger(), "Passing new path to controller.");

  setPlannerPath(path);
}

void MoveBase::publishVelocity(const geometry_msgs::msg::TwistStamped& velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && this->count_subscribers(vel_publisher_->get_topic_name()) > 0)
  {
    vel_publisher_->publish(std::move(cmd_vel));
  }
}

void MoveBase::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = controller_costmap_ros_->getBaseFrameID();
  velocity.header.stamp = now();
  publishVelocity(velocity);
}

bool MoveBase::isGoalReached()
{
  geometry_msgs::msg::PoseStamped pose;

  if (!getRobotPose(pose))
  {
    return false;
  }

  nav_2d_msgs::msg::Twist2D twist = getThresholdedTwist(odom_sub_->getTwist());
  geometry_msgs::msg::Twist velocity = nav_2d_utils::twist2Dto3D(twist);
  return goal_checker_->isGoalReached(pose.pose, end_pose_, velocity);
}

bool MoveBase::getRobotPose(geometry_msgs::msg::PoseStamped& pose)
{
  geometry_msgs::msg::PoseStamped current_pose;
  if (!controller_costmap_ros_->getRobotPose(current_pose))
  {
    return false;
  }
  pose = current_pose;
  return true;
}

}  // namespace move_base