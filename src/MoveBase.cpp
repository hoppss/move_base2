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
  , default_ids_{ "GridBased" }
  , default_types_{ "nav2_navfn_planner/NavfnPlanner" }
  , global_costmap_(nullptr)
{
  RCLCPP_INFO(get_logger(), "Creating");

  for (size_t i = 0; i < default_ids_.size(); ++i)
  {
    RCLCPP_INFO(get_logger(), "default_ids_[%d]: %s", i, default_ids_[i].c_str());  // GridBased
    RCLCPP_INFO(get_logger(), "default_types_[%d] : %s", i,
                default_types_[i].c_str());  // nav2_navfn_planner/NavfnPlanner
  }

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);

  if (planner_ids_ == default_ids_)
  {
    for (size_t i = 0; i < default_ids_.size(); ++i)
    {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
      RCLCPP_INFO(get_logger(), "default_type[%i] : %s", i,
                  default_types_[i].c_str());  // nav2_navfn_planner/NavfnPlanner
    }
  }

  // Setup the global costmap
  RCLCPP_INFO(get_logger(), "what is namespace: %s", get_namespace());
  global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "global_costmap", std::string{ get_namespace() }, "global_costmap");

  // Launch a thread to run the costmap node, it's singleThreadExecutor
  global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);

  // Setup the local costmap
}

MoveBase::~MoveBase()
{
  planners_.clear();
  global_costmap_thread_.reset();
}

nav2_util::CallbackReturn MoveBase::on_configure(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Configuring");

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

  // Create the action servers for path planning to a pose and through poses
  service_ = this->create_service<move_base2::srv::NavigateToPose>(
      "/NaviTo", std::bind(&MoveBase::handleService, this, std::placeholders::_1, std::placeholders::_2));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_activate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Activating");

  global_costmap_ros_->on_activate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->activate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_deactivate(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  global_costmap_ros_->on_deactivate(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->deactivate();
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MoveBase::on_cleanup(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  tf_.reset();
  global_costmap_ros_->on_cleanup(state);

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it)
  {
    it->second->cleanup();
  }
  planners_.clear();
  global_costmap_ = nullptr;

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
  (void)request;
  (void)response;
}

}  // namespace move_base
