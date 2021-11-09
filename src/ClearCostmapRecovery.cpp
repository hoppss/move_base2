#include "move_base2/ClearCostmapRecovery.hpp"


namespace move_base
{

ClearCostmapRecovery::ClearCostmapRecovery()
: logger_(rclcpp::get_logger("move_base_clear_costmap_recovery"))
{
}

ClearCostmapRecovery::~ClearCostmapRecovery() {}

void ClearCostmapRecovery::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent)
{
  node_ = parent;

  global_except_client_ = node_->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>(
    "global_costmap/clear_except_global_costmap");
  local_except_client_ = node_->create_client<nav2_msgs::srv::ClearCostmapExceptRegion>(
    "local_costmap/clear_except_global_costmap");
}

bool ClearCostmapRecovery::clearExceptRegion(bool is_global, double reset_distance)
{

  if (is_global) {

    if (!global_except_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(logger_, "failed to find clear_except_global_costmap server");
      return false;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearCostmapExceptRegion::Request>();
    request->reset_distance = reset_distance;

    using ServiceResponseFuture =
      rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedFuture;

    bool ok = false;
    auto response_received_callback = [this, &ok](ServiceResponseFuture future) {
        RCLCPP_INFO(logger_, "clear_except_global_costmap client success");
        ok = true;
      };

    auto future_result = global_except_client_->async_send_request(
      request,
      response_received_callback);
    int j = 0;
    while (!ok && j++ < 50) { // wait for 5s
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (j % 10 == 0) {
        RCLCPP_INFO(logger_, "wait for %d's result: %d s! ", j / 10);
      }
    }

    if (!ok) {
      RCLCPP_ERROR(logger_, "clear_except_global_costmap service call failed");
      return false;
    }
    return true;

  } else {
    if (!local_except_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(logger_, "failed to find clear_except_local_costmap");
      return false;
    }

    auto request = std::make_shared<nav2_msgs::srv::ClearCostmapExceptRegion::Request>();
    request->reset_distance = reset_distance;

    bool ok = false;
    using ServiceResponseFuture =
      rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedFuture;
    auto response_received_callback = [this, &ok](ServiceResponseFuture future) {
        RCLCPP_INFO(logger_, "clear_except_global_costmap client success");
        ok = true;
      };

    auto future_result = local_except_client_->async_send_request(
      request,
      response_received_callback);

    int j = 0;
    while (!ok && j++ < 50) { // wait for 5s
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (j % 10 == 0) {
        RCLCPP_INFO(logger_, "wait for %d's result: %d s! ", j / 10);
      }
    }

    if (!ok) {
      RCLCPP_ERROR(logger_, "clear_except_local_costmap service call failed");
      return false;
    }
    return true;
  }
}
} // namespace
