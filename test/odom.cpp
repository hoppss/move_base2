#include <iostream>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
  {
    // subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom_out", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));


    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom_out", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  };

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    rclcpp::Time t = rclcpp::Time(msg->header.stamp);
    double dt = t.seconds() - last_stamp_.seconds();
    if (dt > 0.1)
    {
      RCLCPP_ERROR(this->get_logger(), "dt %f, speed[%f, %f]", dt, msg->twist.twist.linear.x,
                   msg->twist.twist.angular.z);
    }
    else
    {
    }

    last_stamp_ = t;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Time last_stamp_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();

  return 0;
}