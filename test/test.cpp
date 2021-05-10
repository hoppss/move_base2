#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/local_costmap/costmap", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  };

private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "callback %ld", msg->header.stamp.nanosec);
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}