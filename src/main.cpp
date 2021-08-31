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
#include <memory>

#include "move_base2/MoveBase.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // setvbuf(stdout, nullptr, _IONBF, BUFSIZ);
  // setvbuf(stdout, NULL, _IOLBF, 0);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<move_base::MoveBase>();

  // node->loop();
  // rclcpp::spin(node->get_node_base_interface());

  rclcpp::executors::MultiThreadedExecutor exec_(rclcpp::ExecutorOptions(), 2);
  exec_.add_node(node->get_node_base_interface());
  exec_.spin();

  rclcpp::shutdown();

  return 0;
}
