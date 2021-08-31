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
#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "automation_msgs/srv/navigate_to_pose.hpp"


using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: navi_to_client x_coord y_coord");
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);  // stod, char* to double
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "x[%f], y[%f]", x, y);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("navi_to_client");
  rclcpp::Client<automation_msgs::srv::NavigateToPose>::SharedPtr client =
    node->create_client<automation_msgs::srv::NavigateToPose>("/NaviTo");

  auto request = std::make_shared<automation_msgs::srv::NavigateToPose::Request>();

  request->is_cancel = false;
  // request->mode = automation_msgs::srv::NavigateToPose::Request::NAVI_MODE;
  request->planner_id = "GridBased";
  request->controller_id = "FollowPath";

  // request->planner_id = "Straight2D";      // Straight2D, GridBased
  // request->controller_id = "PurePersuit";  // PurePersuit; FollowPath
  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = x;
  request->goal.pose.position.y = y;
  request->goal.pose.orientation.w = 1.0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "client result~!");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  rclcpp::shutdown();
  return 0;
}
