#ifndef _SPEAKER_CLIENT_H_
#define _SPEAKER_CLIENT_H_

#include <iostream>
#include <string>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// #include "interaction_msgs/srv/camera_service.hpp"
// #include "interaction_msgs/srv/face_manager.hpp"
// #include "interaction_msgs/msg/body_info.hpp"
// #include "interaction_msgs/msg/face_info.hpp"
// #include "interaction_msgs/msg/face_result.hpp"
#include "interaction_msgs/action/audio_play.hpp"

namespace move_base
{
enum FollowType
{
  FAR_WAIT = 0,    // 620
  FAST_WAIT,       // 621
  FLOW_LOST,       // 622
  CHANGE_ROUTE,    // 623
  OBSTACLE,        // 624
  KEEP_DISTANCE,   // 625
  EMERGENCY_STOP,  // 626
  LAST_FLAG,       // flag for size
};

// singleton mode
class SpeakerClient
{
public:
  using AudioPlayT = interaction_msgs::action::AudioPlay;
  using GoalHandleAudio = rclcpp_action::ClientGoalHandle<AudioPlayT>;

  SpeakerClient();
  ~SpeakerClient();

  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& parent);
  bool play(FollowType type);

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_;
  std::string name_;

  bool sendGoal(int audio_id);
  void goal_response_callback(std::shared_future<GoalHandleAudio::SharedPtr> future);
  void feedback_callback(GoalHandleAudio::SharedPtr, const std::shared_ptr<const AudioPlayT::Feedback> feedback);
  void result_callback(const GoalHandleAudio::WrappedResult& result);

  rclcpp_action::Client<AudioPlayT>::SharedPtr audio_client_;
};

}  // namespace move_base

#endif
