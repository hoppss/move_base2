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

#ifndef MOVE_BASE2__REQUEST_INFO_HPP_
#define MOVE_BASE2__REQUEST_INFO_HPP_

#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"

struct requestInfo
{
  geometry_msgs::msg::PoseStamped goal;
  std::string planner_id;
  std::string controller_id;
};

#endif  // MOVE_BASE2__REQUEST_INFO_HPP_
