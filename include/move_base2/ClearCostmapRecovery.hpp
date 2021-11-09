#ifndef MOVEBASE_CLEARCOSTMAP_H__
#define MOVEBASE_CLEARCOSTMAP_H__

#include <string>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

namespace move_base
{
class ClearCostmapRecovery {
public:

  using SharedPtr = std::shared_ptr<ClearCostmapRecovery>;

  explicit ClearCostmapRecovery();
  ~ClearCostmapRecovery();

  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr & parent);

  bool clearExceptRegion(bool is_global, double reset_distance);

  private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_;


  //except-region clients
  std::shared_ptr<rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>> global_except_client_;
  std::shared_ptr<rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>> local_except_client_;
};

} // namespace

#endif
