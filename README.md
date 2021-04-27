# 仿真

## 启动tb3 gazebo

```
ros2 launch  turtlebot3_gazebo turtlebot3_world.launch.py
```

## 启动定位
```
ros2 launch  move_base2 localization_launch.py
```

## 启动move_base
```
ros2 run move_base2 move_base_node --ros-args --params-file ~/dd_ws/src/move_base2/params/nav2_params.yaml
```
## or
```
ros2 launch move_base2 navigation_launch.py
```

## rviz2

- 启动rviz2
- 先给amcl 发布一个初始位置

## lifecycle 激活
```
ros2 run move_base2  lifecycle_client
```
> active 完成后， ctrl+c

## 发布目标点请求
```
ros2 run move_base2 navi_to_client 1.5 0.5
```

> 在rviz2 上观察坐标， 命令行参数后两个是坐标x，y; 忽略角度信息，默认对齐都是map frame