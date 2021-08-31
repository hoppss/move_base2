# MoveBase2
---------

## Introduction
Like move_base, the move_base2 package provides an implementation of an service/ a topic that, given a goal in the world, will attempt to reach it with a mobile base. The move_base2 node links together a global and local controller to accomplish its global navigation task. It supports any global planner adhering to the nav2_core::GlobalPlanner interface specified in the nav2_core package and any local controller adhering to the nav2_core::Controller interface specified in the nav2_core package. The move_base2 node also maintains two costmaps, one for the global planner, and one for a local controller (see the nav2_costmap_2d package) that are used to accomplish navigation tasks.

On the basis of MoveBase, MoveBase2 retains the basic navigation framework, replacing Actionlib with ROS2 Service (the most ideal situation is Action) to solve the asynchronous navigation problem from the ROS system layer. At the same time, specific collision detection and recovery behaviors are formulated for the tracking mode.

## Usage Introduction

### Start the navigation node
See [README.md](https://partner-gitlab.mioffice.cn/limao1/move_base2/-/blob/master/README.md)

```
    ros2 launch move_base2 navigation_launch_tb3.launch
```
Or

```
    ros2 launch move_base2 tracking_mode_mb.launch
```

This Only starts navigation, can subscribe/publish servieces and topics

### Published Topics
The published topics differ according to the device and parameters. After running the above command, the following list of topics will be available (This is a partial list. For full one type ros2 topic list):

- /body_cmd: motion_msgs/msg/SE3VelocityCMD
- /cmd_vel: geometry_msgs/msg/Twist
- /global_plan: nav_msgs/msg/Path
- /local_plan: nav_msgs/msg/Path
- /move_base_status: automation_msgs/msg/NavStatus
- /plan: nav_msgs/msg/Path

### Available Services
- /NaviTo: Usage for receiving pose to navigate. Usage:
```
ros2 service call /NaviTo automation_msgs/srv/NavigateToPose "{}"
```
- /get_mode: Change navigation mode between NavMode_Track and NavMode_NAV_AB. Usage:
```
ros2 service call /get_mode automation_msgs/srv/NavMode "{sub_mode: 4}"
```
- /move_base_node/change_state: Bringup the navigation stack. Usage:
```
ros2 service call lifecycle_msgs/srv/ChangeState "{trasition: {id=1}}"
ros2 service call lifecycle_msgs/srv/ChangeState "{trasition: {id=3}}"
```

### Subscribed Topics
- /ObstacleDetection: Used for emergency obstacle avoidance and trapped of machines.
- /odom_out: Used for localization in planing and controlling.
- /tracking_pose: Used for receiving pose to track.

### Launch parameters
The following parameters are available by the wrapper:
- namespace:  top-level namespace
- use_sim_time:   use simulation(Gazebo) clock if true
- autostart:  automatically startup the nav2 stack
- nav_params_file:    full path to the ROS2 parameters file to use
- map_subscribe_transient_local:  whether to set the map subscriber QoS to transient local

## Install
Dependencies:
    navigation2, athena_interface(automation_msgs interaction_msgs athna_utils ception_msgs)

* Install Navigation2 Stack
    ```
    $ sudo apt install ros-<distro>-navigation2 ros-<distro>-nav2-bringup
    ```

* Install Athena Interfaces by Sources
    ```
    $ mkdir -p ~/<workspace dir>/src
    $ cd ~/<workspace dir>/src
    $ git clone git@partner-gitlab.mioffice.cn:limao1/athena_interfaces.git
    $ cd ~/<workspace dir>
    $ colcon build --merge-install
    ```

* Install MoveBase2
    ```
    $ cd ~/<workspace dir>/src
    $ git clone git@partner-gitlab.mioffice.cn:limao1/move_base2.git
    $ cd ..
    $ colcon build --merge-install --packages-select move_base2
    ```

## License
```
    // Cop// Copyright (c) 2021 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
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
```
