#ifndef RECEIVE_GOAL_MB_HPP_
#define RECEIVE_GOAL_MB_HPP_

#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav2_util/lifecycle_node.hpp>

#include "automation_msgs/srv/navigate_to_pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nav2_receive_goal
{
// class ReceiveGoalMb : public nav2_util::LifecycleNode
class ReceiveGoalMb : public rclcpp::Node
{
public:
  ReceiveGoalMb();

  ~ReceiveGoalMb();

  // nav2_util::CallbackReturn on_configure (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_activate (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_deactivate (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_cleanup (const rclcpp_lifecycle::State & state) override;
  // nav2_util::CallbackReturn on_shutdown (const rclcpp_lifecycle::State & state) override;

private:
  void srcPoseHandle(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  bool transformPose(const std::string& target_frame, const geometry_msgs::msg::PoseStamped& in_pose,
                     geometry_msgs::msg::PoseStamped& out_pose);

  // rclcpp_lifecycle::LifecyclePublisher< geometry_msgs::msg::PoseStamped>::SharedPtr tar_pose_pub_;
  //   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tar_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr src_pose_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string target_frame_;

  // double transform_tolerance_;  // set in_pose stamp to zero, use zero timeout for latest tf data

  bool start_tracking_;
  geometry_msgs::msg::PoseStamped prev_pose_;

  rclcpp::Client<automation_msgs::srv::NavigateToPose>::SharedPtr navi_to_client_;
  std::shared_ptr<automation_msgs::srv::NavigateToPose::Request> req_;

  std::mutex mutex_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_vec_;
  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr tracking_marker_pub_;
  void publishMarker(geometry_msgs::msg::PoseStamped& pose, int type = 1);
};

}  // namespace nav2_receive_goal

#endif  // RECEIVE_GOAL_MB_HPP_
