#ifndef __REPORTER__
#define __REPORTER__

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "automation_msgs/msg/nav_status.hpp"

namespace move_base
{
class Reporter
{
public:
  Reporter();
  ~Reporter();

  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent);
  void report(const int mode, const uint8_t status, std::string description);

private:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<automation_msgs::msg::NavStatus>::SharedPtr reporter_pub_;
};

}  // namespace move_base

#endif