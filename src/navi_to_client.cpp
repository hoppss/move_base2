#include "rclcpp/rclcpp.hpp"

#include "move_base2/srv/navigate_to_pose.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("navi_to_client");
  rclcpp::Client<move_base2::srv::NavigateToPose>::SharedPtr client =
      node->create_client<move_base2::srv::NavigateToPose>("/NaviTo");

  auto request = std::make_shared<move_base2::srv::NavigateToPose::Request>();

  request->is_cancel = false;
  request->mode = move_base2::srv::NavigateToPose::Request::NAVI_MODE;
  request->planner_id = "GridBased";
  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = 1.5;
  request->goal.pose.position.y = 0.5;
  request->goal.pose.orientation.w = 1.0;

  while (!client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client result~!");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}