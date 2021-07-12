#include "move_base2/Reporter.h"

namespace move_base
{
Reporter::Reporter() : name_("reporter"), logger_(rclcpp::get_logger("move_base_reporter"))
{
}

Reporter::~Reporter()
{
  reporter_pub_.reset();
}

void Reporter::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent)
{
  node_ = parent;
  reporter_pub_ = node_->create_publisher<automation_msgs::msg::NavStatus>("move_base_status", 10);
  reporter_pub_->on_activate();
}

void Reporter::report(const int mode, const uint8_t status, std::string description)
{
  automation_msgs::msg::NavStatus msg;
  msg.mode = mode;
  msg.status = status;
  msg.description = description;

  reporter_pub_->publish(msg);
}

}  // namespace move_base