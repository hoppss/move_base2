#include "move_base2/receive_goal_to_mb.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/execution_timer.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/create_timer_ros.h"

#include <iostream>
#include <iomanip>

namespace nav2_receive_goal
{
ReceiveGoalMb::ReceiveGoalMb() : rclcpp::Node("receive_goal_to_mb"), start_tracking_(false)
//: nav2_util::LifecycleNode("receive_goal", "", true)
{
  declare_parameter("target_frame", rclcpp::ParameterValue("map"));
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(1.0));
  get_parameter("target_frame", target_frame_);
  get_parameter("transform_tolerance", transform_tolerance_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface =
      std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  src_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "tracking_pose", sub_qos, std::bind(&ReceiveGoalMb::srcPoseHandle, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&ReceiveGoalMb::timerCallback, this));

  // tar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tar_pose", rclcpp::SystemDefaultsQoS());
  req_ = std::make_shared<move_base2::srv::NavigateToPose::Request>();
  navi_to_client_ = this->create_client<move_base2::srv::NavigateToPose>("NaviTo");
}

ReceiveGoalMb::~ReceiveGoalMb()
{
  timer_->cancel();
  navi_to_client_.reset();
  src_pose_sub_.reset();

  req_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
}

// nav2_util::CallbackReturn
// ReceiveGoalMb::on_configure (const rclcpp_lifecycle::State & state) {
//
//    get_parameter("target_frame", target_frame_);
//    get_parameter("transform_tolerance", transform_tolerance_);
//
//    tf_buffer_ = std::make_shared< tf2_ros::Buffer>(rclcpp_node_->get_clock());
//    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
//        rclcpp_node_->get_node_base_interface(),
//        rclcpp_node_->get_node_timers_interface());
//    tf_buffer_->setCreateTimerInterface(timer_interface);
//    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//
//    src_pose_sub_ = create_subscription< geometry_msgs::msg::PoseStamped>(
//        "src_pose",
//        rclcpp::SystemDefaultsQoS(),
//        std::bind(&ReceiveGoalMb::srcPoseHandle, this, std::placeholders::_1));
//
//    tar_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("tar_pose", rclcpp::SystemDefaultsQoS());
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_activate (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_->on_activate();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_deactivate (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_->on_deactivate();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_cleanup (const rclcpp_lifecycle::State & state) {
//    tar_pose_pub_.reset();
//}
//
// nav2_util::CallbackReturn
// ReceiveGoalMb::on_shutdown (const rclcpp_lifecycle::State & state) {
//    RCLCPP_INFO(get_logger(), "Shutting down");
//    return nav2_util::CallbackReturn::SUCCESS;
//}

void ReceiveGoalMb::srcPoseHandle(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "tracking_pose, in");
  geometry_msgs::msg::PoseStamped tar_pose;
  if (false == transformPose(target_frame_, *msg, tar_pose))
  {
    RCLCPP_ERROR(this->get_logger(), "tracking_pose, transform pose failed");
    return;
  }

  std::lock_guard<std::mutex> guard(mutex_);
  goals_vec_.push_back(tar_pose);
}

bool ReceiveGoalMb::transformPose(const std::string& frame, const geometry_msgs::msg::PoseStamped& in_pose,
                                  geometry_msgs::msg::PoseStamped& out_pose)
{
  if (in_pose.header.frame_id == frame)
  {
    out_pose = in_pose;
    return true;
  }

  try
  {
    tf_buffer_->transform(in_pose, out_pose, frame, tf2::durationFromSec(transform_tolerance_));
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "Exception in transformPose: %s", ex.what());
    return false;
  }
}

void ReceiveGoalMb::timerCallback()
{
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *get_clock(), 2000, "receive_goal ...");

  if (goals_vec_.empty())
  {
    return;
  }

  geometry_msgs::msg::PoseStamped goal;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal = goals_vec_.back();  // otain latest msg
    goals_vec_.clear();        // clear old data
  }

  if (nav2_util::geometry_utils::euclidean_distance(goal.pose, prev_pose_.pose) > 0.2)
  {
    //
    req_->planner_id = "Straight2D";
    req_->controller_id = "PurePersuit";
    req_->goal = goal;

    prev_pose_ = goal;  // save last tracking goal

    // define async callback function
    auto response_received_callback = [this](rclcpp::Client<move_base2::srv::NavigateToPose>::SharedFuture result) {
      auto response = result.get();
      if (response->result == true && rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(), "client callback success %s.", response->description.c_str());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "client callback failed %s.", response->description.c_str());
      }
    };

    if (navi_to_client_->wait_for_service(std::chrono::milliseconds(100)))
    {
      auto future = navi_to_client_->async_send_request(req_, response_received_callback);
    }
  }
}

}  // namespace nav2_receive_goal
