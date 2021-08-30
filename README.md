# MoveBase2
---------

## 介绍
move_base2 该软件包和ros1 move_base 功能类似，实现了基于话题或者服务接口的导航功能。该软件包可加载两张代价地图costmap，全局规划器和局部控制器插件，插件的需要严格按照nav2_core 接口进行继承，可通过配置文件加载指定的pluginlib， 支持多规划器和多控制器并在发起任务时选择指定插件。
根据当前的业务分成了两种模式，跟随模式和AB点导航模式，跟随模式通过话题/tracking_pose 持续接受相机坐标系位姿，AB点导航通过服务/NaviTo接受任务。相应状态通过话题/move_base_status 给出。

## 接口
### 话题发布接口
主要发布的ros2 话题列表如下所示， 实际真机上可能名称空间前面有namespace

- /body_cmd: motion_msgs/msg/SE3VelocityCMD
- /cmd_vel: geometry_msgs/msg/Twist
- /global_plan: nav_msgs/msg/Path
- /local_plan: nav_msgs/msg/Path
- /move_base_status: automation_msgs/msg/NavStatus
- /plan: nav_msgs/msg/Path

### 服务接口
- /NaviTo: 接受AB点导航指令， 接受某个坐标系下具体位姿
```
ros2 service call /NaviTo automation_msgs/srv/NavigateToPose "{...}"
```
- /get_mode: 设置模式，具体有NavMode_Track/NavMode_AB两个模式，默认为NavMode_AB
> 服务内容定义详见 athena_interfaces/automation_msgs/srv/NavigateToPose.srv
```
ros2 service call /get_mode automation_msgs/srv/NavMode "{sub_mode: 4}"
```
- /move_base_node/change_state: 激活该节点lifecycle
```
ros2 service call lifecycle_msgs/srv/ChangeState "{trasition: {id=1}}"
ros2 service call lifecycle_msgs/srv/ChangeState "{trasition: {id=3}}"
```

### 话题订阅接口
- /ObstacleDetection: 超声障碍物检测
- /odom_out: 腿关节和imu融合的里程计
- /tracking_pose: 目标跟踪位姿

### launch启动参数
launch启动时关键参数:
- namespace:  top-level namespace
- use_sim_time:   use simulation(Gazebo) clock if true
- autostart:  automatically startup the nav2 stack
- nav_params_file:    full path to the ROS2 parameters file to use
- map_subscribe_transient_local:  whether to set the map subscriber QoS to transient local

## 使用介绍

### 仿真使用

```
    # 启动tb3仿真器
    ros2 launch  turtlebot3_gazebo turtlebot3_world.launch.py

    # amcl全局定位
    ros2 launch  move_base2 localization_launch.py

    # amcl 或者 启动 slam
    ros2 launch slam_toolbox online_async_launch.py

    ## amcl 或者 启动slam 并 后续保存地图等
    ros2 run nav2_map_server map_saver_cli -f ~/map

    # 启动导航
    ros2 launch move_base2 navigation_launch_tb3.py

    # lifecycle 激活
    ros2 run move_base2  lifecycle_client
```

### 真机使用

```
    # 其他模块将bringup 一起开机启动
    # 真机启动对应launch文件为
    ros2 launch move_base2 tracking_mode_mb.launch
```

## rviz2发布目标点

> 新设计rviz2 Tools发布目标点插件rviz2_plugins
> 新打开rviz2，在菜单栏右侧，点击 + ， 选择 "NaviToGoalTool"
> move_base2 使用了/NaviTo service [move_base2::srv/NavigateToPose.srv]