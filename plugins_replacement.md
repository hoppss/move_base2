# move_base2


## 特性

    -   内部planner/controller, 要求延用了nav2_core， pluginlib 等基类和编程范式
    -   内部使用原生costmap
    -   内部支持多个planner, 多个controller 实例化，通过id 进行区分， 类似nav2_controller， nav2_planner 的设计
    -   导航接口可指定planner_id， controller_id
    -   参数配置在./param/nav2_params_dc.yaml， 全局规划配置参考GridBased, 局部规划配置参考FollowPath， 并行在下面增加即可
    -   AB导航通过rviz2 Tool 插件触发，接口是服务形式，/NaviTo [automation_msgs/srv/NavigateToPose.srv]
    -   跟随功能，接口形式是服务，/tracking_pose [geometry_msgs/msg/PoseStamped], 内部回调函数trackingPoseCallback(), 会自动填充请求的planner_id， controller_id 这块是写死的， 如需修改，简单改一下这两个热id字段即可