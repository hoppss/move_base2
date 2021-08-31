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
#include <string>
#include "move_base2/Reporter.hpp"

namespace move_base
{
Reporter::Reporter()
: name_("reporter"), logger_(rclcpp::get_logger("move_base_reporter"))
{
}

Reporter::~Reporter()
{
  reporter_pub_.reset();
}

void Reporter::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent)
{
  node_ = parent;
  reporter_pub_ = node_->create_publisher<automation_msgs::msg::NavStatus>("move_base_status", 10);
  reporter_pub_->on_activate();
}

void Reporter::report(const int mode, const uint8_t status, std::string description)
{
  automation_msgs::msg::NavStatus msg;
  msg.mode = mode;
  msg.status = status;
  msg.description = description;

  reporter_pub_->publish(msg);
  RCLCPP_INFO(
    logger_, "mode %d, status %d, descript %s", static_cast<int>(mode),
    static_cast<int>(status), description.c_str());
}

}  // namespace move_base
