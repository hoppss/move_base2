## 仿真启动tb3 gazebo

```
ros2 launch  turtlebot3_gazebo turtlebot3_world.launch.py
```

## 启动定位, amcl + known map
```
ros2 launch  move_base2 localization_launch.py
```
> rviz2 发布初始位置

or
## slam
```
ros2 launch slam_toolbox online_async_launch.py
## save map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 仿真情況下启动move_base
```
## 命令行加载
ros2 run move_base2 move_base_node --ros-args --params-file ~/dd_ws/src/move_base2/params/nav2_params.yaml
```
or
```
## 使用dwb
ros2 launch move_base2 navigation_launch.py
```
or
```
## 使用ducheng的planner
ros2 launch move_base2 navigation_launch_dc.py
```

## move_base2 lifecycle 激活
```
ros2 run move_base2  lifecycle_client
```
> active 完成后， ctrl+c

## 发布目标点请求
```
#1
ros2 run move_base2 navi_to_client 1.5 0.5
#2
scripts/publisher.py
#3 rviz2_plugin 切換NaviTo 插件， 實際爲 /NaviTo client
rviz2
```

## 启动实体机器人測試，與杜晨對接

```
## 底盘 + rtabmap location  + object_tracking
## run.py
ros2 launch move_base2 tracking_base.py
## 导航模块
ros2 launch move_base2 tracking_launch_mb.py
```

## ducheng, 實際測試 tracking task

```
cd ros2_foxy
ros2 launch move_base2 tracking_base.py
ros2 launch move_base2 tracking_launch_mb.py
```