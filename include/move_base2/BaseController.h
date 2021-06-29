#ifndef B_C_H_
#define B_C_H_

#include <rclcpp/rclcpp.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace move_base
{
class BaseController
{
public:
  BaseController();
  ~BaseController();

  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent, const std::shared_ptr<tf2_ros::Buffer>& tf,
                  const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher);

  bool rotate(double);

  // tools
  bool getCurrentPose(geometry_msgs::msg::PoseStamped& odom_pose);
  bool transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose);

  bool approachOnlyRotate(const geometry_msgs::msg::PoseStamped& target);

private:
  std::string name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}  // namespace move_base

#endif