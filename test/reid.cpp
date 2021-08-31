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
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interaction_msgs/msg/body.hpp"
#include "interaction_msgs/msg/body_info.hpp"
#include "interaction_msgs/srv/camera_service.hpp"

using namespace std::chrono_literals;  // ms
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<interaction_msgs::msg::BodyInfo>(
      "body", rclcpp::SystemDefaultsQoS(), std::bind(&MinimalSubscriber::body_callback, this, _1));
    timer_ = this->create_wall_timer(300ms, std::bind(&MinimalSubscriber::timer_callback, this));
  }

  void body_callback(const interaction_msgs::msg::BodyInfo::SharedPtr msg)
  {
    last_body_info_ = *msg;
    RCLCPP_INFO(
      this->get_logger(), "cnt %d, size %d", (int)last_body_info_.count,
      (int)last_body_info_.infos.size());
  }

  void timer_callback()
  {
    //
    if (!last_body_info_.infos.empty()) {
      auto tmp = last_body_info_.infos.front();
      if (call(tmp)) {
        timer_->cancel();
      }
    }

    RCLCPP_INFO(get_logger(), "timer");
  }

  bool call(interaction_msgs::msg::Body & bbox)
  {
    std::string service_name = "camera_server";
    auto node = rclcpp::Node::make_shared(service_name + "_client");
    auto client = node->create_client<interaction_msgs::srv::CameraService>("camera_service");

    int i = 0;
    while (!client->wait_for_service(std::chrono::seconds(1)) && ++i < 5) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        return false;
      }
      RCLCPP_INFO(node->get_logger(), "waiting for %s %d seconds", service_name.c_str(), i);
    }

    if (i >= 5) {
      return false;
    }

    auto request = std::make_shared<interaction_msgs::srv::CameraService::Request>();
    request->command = interaction_msgs::srv::CameraService::Request::SET_PARAMETERS;

    char text[100];
    snprintf(
      text, sizeof(text), "reid-bbox=%d,%d,%d,%d", bbox.roi.x_offset, bbox.roi.y_offset,
      bbox.roi.width,
      bbox.roi.height);

    request->args = std::string(text);

    auto result_future = client->async_send_request(request);

    if (rclcpp::spin_until_future_complete(node, result_future, std::chrono::seconds(3)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node->get_logger(), " %s call failed, result %d, msg %s", service_name.c_str(),
        static_cast<int>(result_future.get()->result), result_future.get()->msg);
      return false;
    } else {
      RCLCPP_INFO(
        node->get_logger(), "%s call success, result %d, msg %d", service_name.c_str(),
        static_cast<int>(result_future.get()->result), result_future.get()->msg);
    }
    return true;
  }

private:
  rclcpp::Subscription<interaction_msgs::msg::BodyInfo>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  interaction_msgs::msg::BodyInfo last_body_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}
