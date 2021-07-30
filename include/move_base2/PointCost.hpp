#ifndef POINTCOST_H_
#define POINTCOST_H_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace move_base
{
class PointCost
{
public:
  PointCost();
  ~PointCost() = default;

  void initialize(const nav2_util::LifecycleNode::SharedPtr& nh,
                  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros);

  unsigned char getPointCost(const geometry_msgs::msg::PoseStamped& p);

  bool collisionFreeCheck(const nav_msgs::msg::Path& path, double& sum_dist);  // false is collision

private:
  nav2_util::LifecycleNode::SharedPtr nh_;
  rclcpp::Logger logger_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D* costmap_;

  double check_distance_;
};

}  // namespace move_base

#endif  // POINTCOST_H_