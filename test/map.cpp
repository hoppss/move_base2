#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <chrono>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  };

private:
  void topic_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) const
  {
    rclcpp::Time t = rclcpp::Time(msg->header.stamp);
    double dt = t.seconds() - last_stamp_.seconds();
    RCLCPP_INFO(this->get_logger(), "dt %f", dt);
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;

  rclcpp::Time last_stamp_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}