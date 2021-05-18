# 一、仿真
## 仿真，启动tb3 gazebo

```
ros2 launch  turtlebot3_gazebo turtlebot3_world.launch.py
```

## 仿真，启动定位, amcl + known map
```
ros2 launch  move_base2 localization_launch.py
```
> rviz2 发布初始位置

or
## 仿真，slam
```
ros2 launch slam_toolbox online_async_launch.py
## save map
ros2 run nav2_map_server map_saver_cli -f ~/map
```

## 仿真，启动move_base
```
## 命令行加载
ros2 run move_base2 move_base_node --ros-args --params-file ~/dd_ws/src/move_base2/params/nav2_params.yaml
```
or
```
## 参数为params/nav2_params_dc.yaml， 仿真和实际机器人使用参数一致
ros2 launch move_base2 navigation_launch_dc.py
```

## move_base2 lifecycle 激活
```
ros2 run move_base2  lifecycle_client
```

## rviz2发布目标点
> move_base2 使用了/NaviTo service [move_base2::srv/NavigateToPose.srv]
>
> 新设计了rviz2 发布目标点插件，[rviz2_plugins](https://partner-gitlab.mioffice.cn/limao1/move_base2)
>
> 新打开rviz2，在菜单栏右侧，点击 + ， 选择 NaviToGoalTool
>
> 导航可参考的rviz配置文件可参考./rviz/*.rviz， 一个为仿真用，一个为实际机器人用;
```
rviz2
```


# 二、实际机器人运行

## 启动导航包launch
```
# 01. 先启动别的部分， tracking_base.py 是我自己搞的， 以系统为准
# ros2 launch move_base2 tracking_base.py
# 02. 启动导航launch
ros2 launch move_base2 tracking_launch_mb.py
```
## lifecycle激活， 见仿真对应部分
## rviz2 发送请求， 见仿真对应部分
