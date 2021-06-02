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
  : nav2_util::LifecycleNode("move_base_node", "", true)

  , state_(NavState::UNACTIVE)
  , is_cancel_(false)
  , is_shutdown_(false)

  , navi_mode_(NavMode::NavMode_AB)

  , gp_loader_("nav2_core", "nav2_core::GlobalPlanner")
  , default_planner_ids_{ "GridBased" }
  , default_planner_types_{ "nav2_navfn_planner/NavfnPlanner" }
  , global_costmap_(nullptr)
  , new_global_plan_(false)
  , run_planner_(false)

  , progress_checker_loader_("nav2_core", "nav2_core::ProgressChecker")
  , default_progress_checker_id_{ "progress_checker" }
  , default_progress_checker_type_{ "nav2_controller::SimpleProgressChecker" }

  , goal_checker_loader_("nav2_core", "nav2_core::GoalChecker")
  , default_goal_checker_id_{ "goal_checker" }
  , default_goal_checker_type_{ "nav2_controller::SimpleGoalChecker" }

  , lp_loader_("nav2_core", "nav2_core::Controller")
  , default_controller_ids_{ "FollowPath" }
  , default_controller_types_{ "teb_local_planner::TebLocalPlannerROS" }
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
  ns_ = get_namespace();
  RCLCPP_INFO(get_logger(), "what is namespace: %s", ns_.c_str());
  global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{ get_namespace() }, "global_costmap");

  // Launch a thread to run the costmap node, it's singleThreadExecutor
  global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);

  // collisionfree check
  point_cost_ = std::make_shared<move_base::PointCost>();

  plan_thread_ = std::make_shared<std::thread>(std::bind(&MoveBase::planThread, this));

  // Setup controller + local_costmap
  RCLCPP_INFO(get_logger(), "Creating Controller");
  for (size_t i = 0; i < default_controller_ids_.size(); ++i)
  {
    RCLCPP_INFO(get_logger(), "default_controller_ids_[%d]: %s", i, default_controller_ids_[i].c_str());  // FollowPath
    RCLCPP_INFO(get_logger(), "default_controller_types_[%d] : %s", i,
                default_controller_types_[i].c_str());  // dwb_core::DWBLocalPlanner
  }

  default_odom_topic_ = "odom_chassis";

  declare_parameter("controller_frequency", 10.0);
  declare_parameter("odom_topic", default_odom_topic_);
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

  spin_thread_ = std::make_shared<std::thread>(std::bind(&MoveBase::spinThread, this));
}  // construnctor

void MoveBase::loop()
{
  // main cycle
  while (rclcpp::ok())
  {
    switch (state_)
    {
      case UNACTIVE: {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "unactive cycle.....");  // ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      break;

      case READY: {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "ready cycle.....");  // ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      break;

      case PLANNING: {
        if (is_cancel_)
        {
          publishZeroVelocity();

          {
            std::unique_lock<std::mutex> lock(planner_mutex_);
            state_ = NavState::READY;
          }
          publishZeroVelocity();
          continue;
        }

        rclcpp::Time t = now();

        if ((t.seconds() - last_nofity_plan_time_.seconds()) > 1.0)
        {
          run_planner_ = true;
          planner_cond_.notify_one();
          last_nofity_plan_time_ = now();
        }

        double time_used = t.seconds() - last_valid_plan_time_.seconds();
        if (time_used > 10.0)
        {
          RCLCPP_WARN(get_logger(), "planning, planner timeout %f, reset status", time_used);
          std::unique_lock<std::mutex> lock(planner_mutex_);
          resetState();
          lock.unlock();
          publishZeroVelocity();
        }
      }
      break;

      case CONTROLLING: {
        // RCLCPP_INFO(get_logger(), "control cycle.....");
        if (is_cancel_)
        {
          std::unique_lock<std::mutex> lock(planner_mutex_);
          state_ = NavState::STOPPING;
          continue;
        }

        // 1. use collsion_detect to decide global replan,
        // 2. code in computeControl()
        // rclcpp::Time t = now();
        // if ((t.seconds() - last_nofity_plan_time_.seconds()) > 1.0)
        // {
        //   run_planner_ = true;
        //   planner_cond_.notify_one();
        //   last_nofity_plan_time_ = now();
        //   RCLCPP_INFO(get_logger(), "control cycle, replan");
        // }

        computeControl();
      }
      break;

      case STOPPING: {
        publishZeroVelocity();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        std::unique_lock<std::mutex> lock(planner_mutex_);
        state_ = NavState::PLANNING;
        last_valid_plan_time_ = now();
      }
      break;

      case EXCEPTION: {
      }
      break;

      default:
        break;
    }  // switch
  }    // while
}

MoveBase::~MoveBase()
{
  RCLCPP_INFO(get_logger(), "DeConstructor of MoveBase2");
  is_shutdown_ = true;
  planner_cond_.notify_one();  // let planner thread exit

  planners_.clear();
  global_costmap_thread_.reset();

  if (spin_thread_)
  {
    spin_thread_->join();
  }

  if (plan_thread_)
  {
    plan_thread_->join();
  }
  point_cost_.reset();
}

nav2_util::CallbackReturn MoveBase::on_configure(const rclcpp_lifecycle::State& state)
{
  // configure planner
  RCLCPP_INFO(get_logger(), "Configuring planner interface");

  global_costmap_ros_->on_configure(state);
  global_costmap_ = global_costmap_ros_->getCostmap();

  RCLCPP_INFO(get_logger(), "Costmap size: %d,%d", global_costmap_->getSizeInCellsX(),
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

  period_ = 1000 / controller_frequency_;

  RCLCPP_INFO(get_logger(), "Controller frequency set to %.4fHz, period %d", controller_frequency_, period_);

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

  get_parameter("odom_topic", default_odom_topic_);
  odom_sub_ = std::make_unique<nav2_util::OdomSmoother>(node, 0.1, default_odom_topic_);
  vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  // get mode interfaces
  get_mode_server_ = this->create_service<athena_interfaces::srv::NavMode>(
      "get_mode", std::bind(&MoveBase::getModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Finally, Create the action servers for path planning to a pose and through poses
  service_handle_ = this->create_service<athena_interfaces::srv::NavigateToPose>(
      "NaviTo", std::bind(&MoveBase::handleService, this, std::placeholders::_1, std::placeholders::_2));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  global_costmap_ros_->on_activate(state);
  plan_publisher_->on_activate();

  point_cost_->initialize(shared_from_this(), global_costmap_ros_);

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

  state_ = READY;

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

void MoveBase::handleService(const std::shared_ptr<athena_interfaces::srv::NavigateToPose::Request> request,
                             std::shared_ptr<athena_interfaces::srv::NavigateToPose::Response> response)
{
  // log
  RCLCPP_INFO(get_logger(),
              "NaviTo: planner [%s] controller [%s], pose [%f, %f], is_cancel [%d],\
              goals_queue_size [%d] ",
              request->planner_id.c_str(), request->controller_id.c_str(), request->goal.pose.position.x,
              request->goal.pose.position.y, (int)request->is_cancel, goals_queue_.size());

  // check is_cancel
  if (request->is_cancel)
  {
    is_cancel_ = true;
    return;
  }

  // check lifecycle
  if (state_ == NavState::UNACTIVE)
  {
    RCLCPP_WARN(get_logger(), "NaviTo: unactive, please active lifecycle");
    response->result = athena_interfaces::srv::NavigateToPose::Response::FAILTURE;
    response->description = "request rejected, lifecycle is not active";
    return;
  }

  if (request->planner_id.empty() || request->controller_id.empty() || request->goal.header.frame_id.empty())
  {
    RCLCPP_WARN(get_logger(), "NaviTo: empty request, ple input planner and controller id");
    response->result = athena_interfaces::srv::NavigateToPose::Response::FAILTURE;
    response->description = "request rejected, empty planner/controller/empty id, check dds?";
    return;
  }

  // push request to queue;
  // planThread pop front request, save as current_request_, and start-plan;
  requestInfo req;
  req.planner_id = request->planner_id;
  req.controller_id = request->controller_id;
  req.goal = request->goal;
  req.goal.header.stamp = rclcpp::Time();

  if (!transformPose("map", req.goal, req.goal))
  {
    RCLCPP_WARN(get_logger(), "NaviTo: request goal transform to map frame failed");
    response->result = athena_interfaces::srv::NavigateToPose::Response::FAILTURE;
    response->description = "request failed, transform goal to map frame failed!";
    return;
  }

  std::string c_name = request->controller_id;
  std::string current_controller;
  if (findControllerId(c_name, current_controller))
  {
    current_controller_ = current_controller;
  }
  else
  {
    std::unique_lock<std::mutex> lock(planner_mutex_);
    state_ = NavState::STOPPING;
    lock.unlock();

    RCLCPP_ERROR(get_logger(), "NaviTo: failed to find controller-id");
    response->result = athena_interfaces::srv::NavigateToPose::Response::FAILTURE;
    response->description = "request rejected, error controller_id";
    return;
  }

  last_valid_plan_time_ = now();
  last_valid_control_time_ = now();

  // idle status
  if (state_ == NavState::READY)
  {
    state_ = PLANNING;

    progress_checker_->reset();
    // publishZeroVelocity();
  }

  // receive new goal ???
  if (state_ == NavState::CONTROLLING)
  {
    // state_ = PLANNING;

    progress_checker_->reset();
    // publishZeroVelocity();
  }

  if (state_ == NavState::STOPPING)
  {
    progress_checker_->reset();
  }

  {
    std::unique_lock<std::mutex> lock(planner_mutex_);
    goals_queue_.push(req);
  }

  last_nofity_plan_time_ = now();
  run_planner_ = true;
  planner_cond_.notify_one();  // before cv.notify(), run_planner_ must be true

  response->result = athena_interfaces::srv::NavigateToPose::Response::SUCCESS;
  response->description = "request received";
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
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();

  try
  {
    // check whether or not update global plan
    static nav_msgs::msg::Path temp_path;
    bool need_update_goal_checker = false;  // local flag

    {
      std::unique_lock<std::mutex> lock(planner_mutex_);

      if (new_global_plan_)
      {
        temp_path = last_global_plan_;
        new_global_plan_ = false;
        need_update_goal_checker = true;
      }
    }

    // update global_path/end_pose/goal_checker, if new global plan
    if (need_update_goal_checker)
    {
      controllers_[current_controller_]->setPlan(temp_path);

      auto end_pose = temp_path.poses.back();
      end_pose.header.frame_id = temp_path.header.frame_id;
      end_pose.header.stamp = rclcpp::Time();
      rclcpp::Duration tolerance(rclcpp::Duration::from_seconds(controller_costmap_ros_->getTransformTolerance()));
      nav_2d_utils::transformPose(tf_, controller_costmap_ros_->getGlobalFrameID(), end_pose, end_pose, tolerance);
      goal_checker_->reset();

      RCLCPP_INFO(get_logger(), "Path end point is (%.2f, %.2f)", end_pose.pose.position.x, end_pose.pose.position.y);
      end_pose_ = end_pose.pose;
    }

    // get current pose
    geometry_msgs::msg::PoseStamped pose;

    if (!getRobotPose(pose))
    {
      throw nav2_core::PlannerException("controller: Failed to obtain robot pose");
    }

    // update progress_checker
    if (!progress_checker_->check(pose))
    {
      throw nav2_core::PlannerException("controller: Failed to make progress");
    }

    // check goal_checker
    if (isGoalReached())
    {
      RCLCPP_INFO(get_logger(), "controller: Reached the goal!");
      state_ = NavState::READY;
      publishZeroVelocity();
      return;
    }

    // whether cycle global plan, notify condition_variable
    rclcpp::Time t = now();
    double sum_dist = 0.0;
    if (!point_cost_->collisionFreeCheck(temp_path, sum_dist))
    {
      if (sum_dist <= 1.0 && sum_dist > 0)
      {
        RCLCPP_WARN(get_logger(), "controller: Pre-ObsDetect, dist %f, STOPPING", sum_dist);
        publishZeroVelocity();
        state_ = NavState::STOPPING;
        return;
      }
      else
      {
        RCLCPP_WARN(get_logger(), "controller: Remote-ObsDetect, dist %f, REPLAN", sum_dist);
        run_planner_ = true;
        planner_cond_.notify_one();
        last_nofity_plan_time_ = t;
      }

      // possible optimal, prune global path
    }

    // get Twist
    geometry_msgs::msg::Twist twist = odom_sub_->getTwist();
    auto cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(pose, twist);

    publishVelocity(cmd_vel_2d);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    double time_span_in_ms = time_span.count() * 1000;

    RCLCPP_INFO(get_logger(), "Publishing velocity at time %.2f [%f,%f], timecost %f", now().seconds(),
                cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.angular.z, time_span_in_ms);

    if (time_span_in_ms >= period_)
    {
      RCLCPP_WARN(get_logger(), "Control loop missed its desired rate of %.4fHz, time_cost %f", controller_frequency_,
                  time_span_in_ms);
    }

    int sleep_time = period_ - time_span_in_ms;
    if (time_span_in_ms < period_ && sleep_time > 10)
      std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
    else
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }
  catch (nav2_core::PlannerException& e)
  {
    RCLCPP_ERROR(this->get_logger(), e.what());
    publishZeroVelocity();
    state_ = NavState::STOPPING;
  }

  RCLCPP_DEBUG(get_logger(), "Controller succeeded, setting result");
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

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());

  auto cmd_vel_2d = controllers_[current_controller_]->computeVelocityCommands(pose, twist);

  // std::shared_ptr<Action::Feedback> feedback = std::make_shared<Action::Feedback>();
  // feedback->speed = std::hypot(cmd_vel_2d.twist.linear.x, cmd_vel_2d.twist.linear.y);
  // feedback->distance_to_goal = nav2_util::geometry_utils::euclidean_distance(end_pose_, pose.pose);
  // action_server_->publish_feedback(feedback);

  RCLCPP_DEBUG(get_logger(), "Publishing velocity at time %.2f", now().seconds());
  publishVelocity(cmd_vel_2d);
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

  geometry_msgs::msg::Twist twist = getThresholdedTwist(odom_sub_->getTwist());
  return goal_checker_->isGoalReached(pose.pose, end_pose_, twist);
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

void MoveBase::planThread()
{
  RCLCPP_INFO(get_logger(), "planThread init");

  while (rclcpp::ok())
  {
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RCLCPP_INFO(get_logger(), "planThread ->");

    // wait 阻塞，释放锁
    // notify 解除阻塞，加锁
    std::unique_lock<std::mutex> lock(planner_mutex_);
    planner_cond_.wait(lock, [this]() { return run_planner_ && !is_cancel_; });

    lock.unlock();

    run_planner_ = false;

    // check for ctrl_c signal
    if (is_shutdown_)
      break;

    // check global state_
    if (state_ == NavState::STOPPING)
      continue;

    RCLCPP_INFO(get_logger(), "planner_thread: getcurrentpose");

    geometry_msgs::msg::PoseStamped start;
    if (!global_costmap_ros_->getRobotPose(start))
    {
      RCLCPP_ERROR(get_logger(), "planner_thread: getcurrentpose failed");
      continue;
    }

    // check queues size, or update current_request_ variable
    RCLCPP_INFO(get_logger(), "planner_thread: queue size %d", goals_queue_.size());
    {
      std::unique_lock<std::mutex> lock(planner_mutex_);
      while (goals_queue_.size() > 1)
        goals_queue_.pop();

      if (goals_queue_.size() == 1)
      {
        RCLCPP_INFO(get_logger(), "update queue goal to plan, pop");
        current_request_ = goals_queue_.front();  // current_request_ only update at this place
        goals_queue_.pop();
      }
    }

    geometry_msgs::msg::PoseStamped goal = current_request_.goal;
    nav_msgs::msg::Path path = getPlan(start, goal, current_request_.planner_id);

    if (path.poses.size() == 0)
    {
      RCLCPP_WARN(get_logger(),
                  "planner_thread: algorithm %s failed to generate a valid"
                  " path to (%.2f, %.2f)",
                  current_request_.planner_id.c_str(), current_request_.goal.pose.position.x,
                  current_request_.goal.pose.position.y);
      // double time_used = now().seconds() - last_valid_plan_time_.seconds();
      // if (time_used > 10.0)
      // {
      //   RCLCPP_WARN(get_logger(), "plan_thread: planner timeout %f, reset status", time_used);
      //   std::unique_lock<std::mutex> lock(planner_mutex_);
      //   resetState();
      //   lock.unlock();

      //   // publishZeroVelocity();
      // }
      continue;
    }
    else
    {
      RCLCPP_INFO(get_logger(), "planner_thread: Found valid path of size %lu to (%.2f, %.2f), publish",
                  path.poses.size(), goal.pose.position.x, goal.pose.position.y);

      publishPlan(path);
      last_global_plan_ = path;
      last_valid_plan_time_ = now();
      new_global_plan_ = true;

      if (state_ == NavState::PLANNING)
      {
        state_ = NavState::CONTROLLING;
        progress_checker_->reset();
      }
    }
  }
}

void MoveBase::spinThread()
{
  // while (rclcpp::ok())
  // {
  rclcpp::spin(this->get_node_base_interface());
  // }
}

void MoveBase::resetState()
{
  new_global_plan_ = false;
  last_global_plan_.poses.clear();
  last_nofity_plan_time_ = now();
  last_valid_plan_time_ = now();
  last_valid_control_time_ = now();
  run_planner_ = false;

  current_request_.planner_id.clear();
  current_request_.controller_id.clear();

  while (!goals_queue_.empty())
    goals_queue_.pop();

  progress_checker_->reset();

  state_ = NavState::READY;
  is_cancel_ = false;
}

void MoveBase::getModeCallback(const std::shared_ptr<athena_interfaces::srv::NavMode::Request> req,
                               std::shared_ptr<athena_interfaces::srv::NavMode::Response> res)
{
  res->success = false;

  if (state_ < NavState::READY)
    return;

  switch (req->control_mode)
  {
    case athena_interfaces::srv::NavMode::Request::EXPLR_NAV_AB: {
      RCLCPP_INFO(this->get_logger(), "Change mode to navigating");
      // config().blackboard->set<std::string>("nav_mode", "navigating");
      if (navi_mode_ != NavMode::NavMode_AB)
        navi_mode_ = NavMode::NavMode_Track;

      setControllerTrackingMode(false);
    }
    break;
    case athena_interfaces::srv::NavMode::Request::TRACK_F:
    case athena_interfaces::srv::NavMode::Request::TRACK_S: {
      RCLCPP_INFO(this->get_logger(), "Change mode to tracking");
      if (navi_mode_ != NavMode::NavMode_Track)
        navi_mode_ = NavMode::NavMode_AB;
      setControllerTrackingMode(true);
      // config().blackboard->set<std::string>("nav_mode", "tracking");
    }
    break;
    case athena_interfaces::srv::NavMode::Request::EXPLR_MAP_UPDATE:
    case athena_interfaces::srv::NavMode::Request::EXPLR_MAP_NEW:
    case athena_interfaces::srv::NavMode::Request::MODE_STOP: {
      RCLCPP_INFO(this->get_logger(), "Stop navigation");
      // config().blackboard->set<std::string>("nav_mode", "none");
    }
    break;
    default:
      return;
  }

  res->success = true;
}

bool MoveBase::setControllerTrackingMode(bool enable)
{
  auto client = this->create_client<rcl_interfaces::srv::SetParameters>("move_base_node/set_parameters");
  client->wait_for_service();

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  rclcpp::Parameter tracking_mode("PurePersuit.tracking_mode", enable);
  request->parameters.push_back(tracking_mode.to_parameter_msg());

  auto future = client->async_send_request(request);
  // rclcpp::spin_until_future_complete(node_, future);

  // RCLCPP_INFO(node_->get_logger(), "Change controller tracking_mode %i",
  //    future.get()->results.front().successful);

  return true;
}

bool MoveBase::transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                             geometry_msgs::msg::PoseStamped& out_pose)
{
  if (in_pose.header.frame_id == target_frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    auto copy_in_pose = in_pose;
    copy_in_pose.header.stamp = rclcpp::Time();
    out_pose = tf_->transform(in_pose, target_frame);
    return true;
  }
  catch (tf2::LookupException& ex)
  {
    RCLCPP_ERROR(get_logger(), "transformPose: No Transform available Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ConnectivityException& ex)
  {
    RCLCPP_ERROR(get_logger(), "transformPose: Connectivity Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::ExtrapolationException& ex)
  {
    RCLCPP_ERROR(get_logger(), "transformPose: Extrapolation Error looking up robot pose: %s\n", ex.what());
  }
  catch (tf2::TimeoutException& ex)
  {
    RCLCPP_ERROR(get_logger(), "transformPose: Transform timeout with tolerance%s", ex.what());
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "transformPose: Failed to transform");
  }

  return false;
}

}  // namespace move_base
