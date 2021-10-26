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
#ifndef MOVE_BASE2__STATE_HPP_
#define MOVE_BASE2__STATE_HPP_

#include <string>
#include <vector>

enum NavState
{
  UNACTIVE = 0,
  READY,
  PLANNING,
  CONTROLLING,
  WAITING,
  TRACKINGROTATERECOVERY,
  EXCEPTION,
  STOPPING,
  SPINRECOVERY,
  BACKUPRECOVERY
};

enum NavMode
{
  NavMode_AB = 0,
  NavMode_Track,
  NavMode_STOP,
};


class RecoveryRecord
{
public:
  RecoveryRecord()
  : cnt_(0) {}

  ~RecoveryRecord()
  {
    clear();
  }

  void clear()
  {
    cnt_ = 0;
    index_.clear();
    name_.clear();
  }

  void update(const std::string & way)
  {
    cnt_++;
    index_.push_back(cnt_);
    name_.push_back(way);
  }

  int cnt_;
  std::vector<int> index_;
  std::vector<std::string> name_;
};

#endif  // MOVE_BASE2__STATE_HPP_
