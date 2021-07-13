# move_base2


## 特性

    -   内部planner/controller, 要求延用了nav2_core， pluginlib 等基类和编程范式
    -   内部支持多个planner, 多个controller 实例化，通过id 进行区分， 类似nav2_controller， nav2_planner 的设计
    -   导航接口可指定planner_id， controller_id
    -   参数配置在./param/nav2_params_dc.yaml， 全局规划类似GridBased, 局部规划类似FollowPath
    -   增加插件后，如果要调用，请修改rviz2_plugins，调整调用的planner_id， 或者controller_id