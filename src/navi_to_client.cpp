#include "rclcpp/rclcpp.hpp"

#include "move_base2/srv/navigate_to_pose.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: navi_to_client x_coord y_coord");
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);  // stod, char* to double
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x[%f], y[%f]", x, y);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("navi_to_client");
  rclcpp::Client<move_base2::srv::NavigateToPose>::SharedPtr client =
      node->create_client<move_base2::srv::NavigateToPose>("/NaviTo");

  auto request = std::make_shared<move_base2::srv::NavigateToPose::Request>();

  request->is_cancel = false;
  request->mode = move_base2::srv::NavigateToPose::Request::NAVI_MODE;
  request->planner_id = "GridBased";
  request->controller_id = "FollowPath";

  // request->planner_id = "Straight2D";      // Straight2D, GridBased
  // request->controller_id = "PurePersuit";  // PurePersuit; FollowPath
  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = x;
  request->goal.pose.position.y = y;
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