#include "move_base2/BaseController.h"

namespace move_base
{
BaseController::BaseController() : name_{ "base_controller" }, logger_(rclcpp::get_logger("teb_logger"))
{
}

BaseController::~BaseController()
{
}

void BaseController::initialize(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent, const std::shared_ptr<tf2_ros::Buffer>& tf,
    const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher)
{
  node_ = parent;

  clock_ = node_->get_clock();

  tf_ = tf;
  vel_pub_ = vel_publisher;
}

bool BaseController::rotate(double angle)
{
  //   geometry_msgs::msg::PoseStamped p_in_b, p_in_o;

  //   tf2::toMsg(tf2::Transform::getIdentity(), p_in_b.pose);
  //   tf2::toMsg(tf2::Transform::getIdentity(), p_in_o.pose);

  //   p_in_b.header.frame_id = "base_footprint";

  //   tf2::Quaternion q;
  //   q.setRPY(0.0, 0.0, angle);
  //   p_in_b.pose.orientation = tf2::toMsg(q);

  //   try
  //   {
  //     // tf_->waitForTransform("odom", "base_footprint", tf2::durationFromSec(1.0));
  //     p_in_o = tf_->transform(p_in_b, "odom", tf2::durationFromSec(0.0));
  //   }
  //   catch (tf2::LookupException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "No Transform available Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::ConnectivityException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Connectivity Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::ExtrapolationException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Extrapolation Error looking up robot pose: %s\n", ex.what());
  //     return false;
  //   }
  //   catch (tf2::TimeoutException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Transform timeout with tolerance");
  //     return false;
  //   }
  //   catch (tf2::TransformException& ex)
  //   {
  //     RCLCPP_ERROR(logger_, "Failed to transform from base to odom");
  //     return false;
  //   }

  double speed = M_PI_4;
  int cnt = static_cast<int>(std::ceil(std::fabs(angle) / speed * 15));

  while (cnt > 0)
  {
    RCLCPP_INFO(logger_, "rotating cnt %d", cnt);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = std::copysign(speed, angle);

    vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    cnt--;
  }

  RCLCPP_INFO(logger_, "rotate finish");
  return true;
}

}  // namespace move_base